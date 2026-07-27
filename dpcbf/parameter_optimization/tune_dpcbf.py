#!/usr/bin/env python3
"""Tune the deployed C++ DPCBF safety filter with reproducible 2D rollouts."""

from __future__ import annotations

import argparse
from contextlib import nullcontext
import copy
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

import yaml

try:
    import optuna
except ImportError:
    optuna = None


# ---------------------------------------------------------------------------
# User configuration
# ---------------------------------------------------------------------------
# One Optuna trial samples one (k_mu, k_lambda, alpha) set. Each sampled set is
# evaluated on `rollouts_per_trial` reproducible environments.
CONFIG: dict[str, Any] = {
    "study": {
        "study_name": "dpcbf_parameter_tuning",
        "num_trials": 100,
        "rollouts_per_trial": 100,
        "evaluation_batch_size": 5,
        "sampler_seed": 42,
        "scenario_seed": 20260727,
        "n_startup_trials": 10,
        "storage": "results/dpcbf_optuna.db",
        "output_dir": "results",
        # SQLite is required to resume an interrupted/continued study.
        "keep_study_database": True,
        # Batch YAML/JSON files are only useful for debugging.
        "keep_trial_artifacts": False,
        "write_study_summary": False,
    },
    "parameter_ranges": {
        "k_mu": {"low": 0.1, "high": 2.0, "log": True},
        "k_lambda": {"low": 0.1, "high": 2.0, "log": True},
        "dpcbf_alpha": {"low": 0.1, "high": 10.0, "log": True},
    },
    "simulation": {
        "episode_duration_s": 20.0,
        "control_dt": 0.002,
        # About 30% of rollouts use canonical constant commands. The others
        # follow interior waypoints with smooth Perlin perturbations.
        "fixed_command_fraction": 0.30,
        "perlin_frequency_hz": 0.12,
        "perlin_velocity_amplitude": 0.35,
        "perlin_yaw_amplitude": 0.35,
        "waypoint_speed": 1.0,
        "waypoint_reached_radius": 0.4,
        "boundary_command_margin": 1.0,
    },
    "obstacles": {
        "count": 20,
        "radius_range": [0.1, 0.5],
        "speed_range": [0.0, 0.8],
    },
    "arena": {
        "size": [10.0, 10.0],
        "center": [0.0, 0.0],
    },
    "robot": {
        "r_rob": 0.25,
        "p_max": 3.0,
        "v_s_max": 2.0,
        "v_s_min": -1.0,
        "v_l_max": 1.0,
        "v_l_min": -1.0,
        "w_max": 1.0,
        "w_min": -1.0,
        "a_s_max": 4.0,
        "a_s_min": -2.0,
        "a_l_max": 2.0,
        "a_l_min": -2.0,
        "s": 1.05,
        "eps_v": 0.05,
        "eps_d": 0.05,
        # Surface-to-surface free space around the initial robot pose.
        # Spawn rule: center_distance >= r_rob + r_obs + spawn_clearance.
        "spawn_clearance": 0.5,
    },
    "qp_parameters": {
        "enabled": True,
        "dpcbf_enabled": True,
        "k_a_s": 1.0,
        "k_a_l": 1.0,
        "default_num_constraints": 10,
        # a_s/a_l are normalized at this reference rate. Their actual per-tick
        # velocity change is multiplied by reference_rate * control_dt.
        "reference_control_frequency_hz": 500.0,
        "ecbf_enabled": True,
        "alpha_ecbf": 1.0,
        "odcbf_enabled": False,
        "decay_weight": 1.0e6,
        "slack_enabled": True,
        "slack_weight": 1.0e6,
    },
    # This is the outer Optuna score, not the inner OSQP objective.
    # Failures dominate tracking performance; all terms are minimized.
    "objective_weights": {
        "failure_rate": 10000.0,
        "lost_survival": 1000.0,
        "tracking_mse": 1.0,
        "intervention_mse": 0.1,
        "slack_mse": 0.001,
        "decay_mse": 0.1,
    },
}


SCRIPT_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_EVALUATOR = (
    REPOSITORY_ROOT / "simulate" / "build" / "dpcbf"
    / "dpcbf_rollout_evaluator"
)


def _resolved_under_script(path_text: str) -> Path:
    path = Path(path_text).expanduser()
    return path if path.is_absolute() else (SCRIPT_DIR / path).resolve()


def _validate_config(config: dict[str, Any]) -> None:
    study = config["study"]
    simulation = config["simulation"]
    robot = config["robot"]
    obstacles = config["obstacles"]
    arena = config["arena"]

    positive_integer_keys = (
        "num_trials",
        "rollouts_per_trial",
        "evaluation_batch_size",
    )
    for key in positive_integer_keys:
        if int(study[key]) <= 0:
            raise ValueError(f"study.{key} must be positive")
    if study["evaluation_batch_size"] > study["rollouts_per_trial"]:
        study["evaluation_batch_size"] = study["rollouts_per_trial"]
    if simulation["control_dt"] <= 0.0 or simulation["episode_duration_s"] <= 0.0:
        raise ValueError("episode_duration_s and control_dt must be positive")
    if not 0.0 <= simulation["fixed_command_fraction"] <= 1.0:
        raise ValueError("fixed_command_fraction must be in [0, 1]")
    if obstacles["count"] <= 0:
        raise ValueError("obstacle count must be positive")
    for key in ("radius_range", "speed_range"):
        lower, upper = obstacles[key]
        if lower < 0.0 or lower > upper:
            raise ValueError(f"invalid obstacles.{key}")
    if min(arena["size"]) <= 0.0:
        raise ValueError("arena dimensions must be positive")
    if robot["spawn_clearance"] < 0.0:
        raise ValueError("spawn_clearance must be nonnegative")
    if robot["s"] <= 1.0:
        raise ValueError("robot.s must be greater than 1")
    for name, spec in config["parameter_ranges"].items():
        if spec["low"] >= spec["high"]:
            raise ValueError(f"invalid parameter range for {name}")
        if spec.get("log", False) and spec["low"] <= 0.0:
            raise ValueError(f"log-scaled range {name} must be positive")


def _trial_yaml(
    config: dict[str, Any],
    parameters: dict[str, float],
    rollout_start_index: int,
    rollout_count: int,
) -> dict[str, Any]:
    robot = copy.deepcopy(config["robot"])
    robot.update(
        {
            "base_body": "pelvis",
            "k_mu": float(parameters["k_mu"]),
            "k_lambda": float(parameters["k_lambda"]),
        }
    )
    qp = copy.deepcopy(config["qp_parameters"])
    qp["alpha"] = float(parameters["dpcbf_alpha"])
    simulation = copy.deepcopy(config["simulation"])
    simulation.update(
        {
            "seed": int(config["study"]["scenario_seed"]),
            "rollout_start_index": int(rollout_start_index),
            "rollouts": int(rollout_count),
        }
    )
    return {
        "optimization_simulation": simulation,
        "dynamic_obstacles": {
            "enabled": True,
            "count": int(config["obstacles"]["count"]),
            "radius_range": list(config["obstacles"]["radius_range"]),
            "speed_range": list(config["obstacles"]["speed_range"]),
            "collision_enabled": True,
            "arena": copy.deepcopy(config["arena"]),
        },
        "robot": robot,
        "qp_parameters": qp,
    }


def _write_yaml(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False)


def _run_evaluator(
    evaluator: Path,
    yaml_path: Path,
    result_path: Path,
) -> dict[str, Any]:
    if not evaluator.is_file():
        raise FileNotFoundError(
            f"C++ evaluator was not found at {evaluator}\n"
            "Build it with:\n"
            "  cmake --build simulate/build --target dpcbf_rollout_evaluator -j"
        )
    completed = subprocess.run(
        [str(evaluator), str(yaml_path), str(result_path)],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "C++ rollout evaluator failed.\n"
            f"stdout:\n{completed.stdout}\n"
            f"stderr:\n{completed.stderr}"
        )
    with result_path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _merge_metrics(
    accumulated: dict[str, Any] | None,
    batch: dict[str, Any],
) -> dict[str, Any]:
    if accumulated is None:
        return copy.deepcopy(batch)
    old_count = int(accumulated["rollouts"])
    batch_count = int(batch["rollouts"])
    total = old_count + batch_count
    count_keys = ("successes", "collisions", "arena_exits", "qp_failures")
    mean_keys = (
        "mean_survival_ratio",
        "mean_tracking_mse",
        "mean_intervention_mse",
        "mean_slack_mse",
        "mean_decay_mse",
        "mean_minimum_clearance",
    )
    accumulated["rollouts"] = total
    for key in count_keys:
        accumulated[key] = int(accumulated[key]) + int(batch[key])
    for key in mean_keys:
        accumulated[key] = (
            float(accumulated[key]) * old_count
            + float(batch[key]) * batch_count
        ) / total
    accumulated["minimum_clearance"] = min(
        float(accumulated["minimum_clearance"]),
        float(batch["minimum_clearance"]),
    )
    return accumulated


def _objective_score(
    metrics: dict[str, Any],
    weights: dict[str, float],
) -> float:
    rollouts = max(int(metrics["rollouts"]), 1)
    failures = (
        int(metrics["collisions"])
        + int(metrics["arena_exits"])
        + int(metrics["qp_failures"])
    )
    failure_rate = failures / rollouts
    return (
        weights["failure_rate"] * failure_rate
        + weights["lost_survival"]
        * (1.0 - float(metrics["mean_survival_ratio"]))
        + weights["tracking_mse"] * float(metrics["mean_tracking_mse"])
        + weights["intervention_mse"]
        * float(metrics["mean_intervention_mse"])
        + weights["slack_mse"] * float(metrics["mean_slack_mse"])
        + weights["decay_mse"] * float(metrics["mean_decay_mse"])
    )


class DpcbfObjective:
    def __init__(
        self,
        config: dict[str, Any],
        evaluator: Path,
        output_dir: Path,
    ) -> None:
        self.config = config
        self.evaluator = evaluator
        self.output_dir = output_dir

    def __call__(self, trial: Any) -> float:
        parameters: dict[str, float] = {}
        for name, spec in self.config["parameter_ranges"].items():
            parameters[name] = trial.suggest_float(
                name,
                float(spec["low"]),
                float(spec["high"]),
                log=bool(spec.get("log", False)),
            )

        rollout_total = int(self.config["study"]["rollouts_per_trial"])
        batch_size = int(self.config["study"]["evaluation_batch_size"])
        accumulated: dict[str, Any] | None = None
        keep_artifacts = bool(
            self.config["study"].get("keep_trial_artifacts", False)
        )
        if keep_artifacts:
            persistent_dir = self.output_dir / f"trial_{trial.number:05d}"
            persistent_dir.mkdir(parents=True, exist_ok=True)
            directory_context: Any = nullcontext(persistent_dir)
        else:
            directory_context = tempfile.TemporaryDirectory(
                prefix=f"dpcbf_trial_{trial.number:05d}_"
            )

        with directory_context as directory:
            trial_dir = Path(directory)
            for batch_index, start in enumerate(
                range(0, rollout_total, batch_size)
            ):
                count = min(batch_size, rollout_total - start)
                yaml_data = _trial_yaml(
                    self.config, parameters, start, count
                )
                yaml_path = trial_dir / f"batch_{batch_index:03d}.yaml"
                result_path = trial_dir / f"batch_{batch_index:03d}.json"
                _write_yaml(yaml_path, yaml_data)
                batch_metrics = _run_evaluator(
                    self.evaluator, yaml_path, result_path
                )
                accumulated = _merge_metrics(accumulated, batch_metrics)
                score = _objective_score(
                    accumulated, self.config["objective_weights"]
                )
                trial.report(score, batch_index)
                if trial.should_prune():
                    if keep_artifacts:
                        partial_path = trial_dir / "partial_summary.json"
                        partial_path.write_text(
                            json.dumps(
                                {
                                    "parameters": parameters,
                                    "score": score,
                                    "metrics": accumulated,
                                },
                                indent=2,
                            ),
                            encoding="utf-8",
                        )
                    raise optuna.TrialPruned()

            assert accumulated is not None
            score = _objective_score(
                accumulated, self.config["objective_weights"]
            )
            if keep_artifacts:
                summary = {
                    "parameters": parameters,
                    "score": score,
                    "metrics": accumulated,
                }
                (trial_dir / "summary.json").write_text(
                    json.dumps(summary, indent=2), encoding="utf-8"
                )
        for key, value in accumulated.items():
            trial.set_user_attr(key, value)
        return score


def _middle_parameters(config: dict[str, Any]) -> dict[str, float]:
    parameters: dict[str, float] = {}
    for name, spec in config["parameter_ranges"].items():
        low = float(spec["low"])
        high = float(spec["high"])
        parameters[name] = (
            math.sqrt(low * high)
            if spec.get("log", False)
            else 0.5 * (low + high)
        )
    return parameters


def evaluate_once(
    config: dict[str, Any],
    evaluator: Path,
    rollout_count: int,
) -> dict[str, Any]:
    parameters = _middle_parameters(config)
    with tempfile.TemporaryDirectory(prefix="dpcbf_smoke_") as directory:
        temporary_dir = Path(directory)
        yaml_data = _trial_yaml(config, parameters, 0, rollout_count)
        yaml_path = temporary_dir / "smoke_test.yaml"
        result_path = temporary_dir / "smoke_test.json"
        _write_yaml(yaml_path, yaml_data)
        metrics = _run_evaluator(evaluator, yaml_path, result_path)
    result = {
        "parameters": parameters,
        "score": _objective_score(metrics, config["objective_weights"]),
        "metrics": metrics,
    }
    print(json.dumps(result, indent=2))
    return result


def run_study(
    config: dict[str, Any],
    evaluator: Path,
    output_dir: Path,
) -> Any:
    if optuna is None:
        raise RuntimeError(
            "Optuna is not installed. Install the tuning dependencies with:\n"
            "  python3 -m pip install -r "
            f"{SCRIPT_DIR / 'requirements.txt'}"
        )
    study_config = config["study"]
    storage_url = None
    if bool(study_config.get("keep_study_database", True)):
        database_path = _resolved_under_script(study_config["storage"])
        database_path.parent.mkdir(parents=True, exist_ok=True)
        storage_url = f"sqlite:///{database_path}"
    sampler = optuna.samplers.TPESampler(
        seed=int(study_config["sampler_seed"]),
        multivariate=True,
    )
    pruner = optuna.pruners.MedianPruner(
        n_startup_trials=int(study_config["n_startup_trials"]),
        n_warmup_steps=1,
        interval_steps=1,
    )
    study = optuna.create_study(
        study_name=str(study_config["study_name"]),
        storage=storage_url,
        load_if_exists=True,
        direction="minimize",
        sampler=sampler,
        pruner=pruner,
    )
    objective = DpcbfObjective(config, evaluator, output_dir)
    study.optimize(
        objective,
        n_trials=int(study_config["num_trials"]),
        n_jobs=1,
        show_progress_bar=True,
    )

    best_parameters = dict(study.best_params)
    best_config = _trial_yaml(
        config,
        best_parameters,
        0,
        int(study_config["rollouts_per_trial"]),
    )
    _write_yaml(output_dir / "best_dpcbf_config.yaml", best_config)
    summary = {
        "study_name": study.study_name,
        "best_trial": study.best_trial.number,
        "best_value": study.best_value,
        "best_parameters": best_parameters,
        "best_user_attributes": study.best_trial.user_attrs,
        "completed_trials": len(
            [
                trial
                for trial in study.trials
                if trial.state == optuna.trial.TrialState.COMPLETE
            ]
        ),
        "pruned_trials": len(
            [
                trial
                for trial in study.trials
                if trial.state == optuna.trial.TrialState.PRUNED
            ]
        ),
    }
    if bool(study_config.get("write_study_summary", False)):
        (output_dir / "study_summary.json").write_text(
            json.dumps(summary, indent=2), encoding="utf-8"
        )
    print(json.dumps(summary, indent=2))
    return study


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Tune k_mu, k_lambda, and DPCBF alpha using C++ OSQP rollouts."
    )
    parser.add_argument(
        "--evaluator",
        type=Path,
        default=DEFAULT_EVALUATOR,
        help="Path to the built dpcbf_rollout_evaluator executable.",
    )
    parser.add_argument(
        "--trials",
        type=int,
        help="Override CONFIG['study']['num_trials'].",
    )
    parser.add_argument(
        "--rollouts",
        type=int,
        help="Override CONFIG['study']['rollouts_per_trial'].",
    )
    parser.add_argument(
        "--evaluate-once",
        action="store_true",
        help="Run midpoint parameters once without requiring Optuna.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    config = copy.deepcopy(CONFIG)
    if args.trials is not None:
        config["study"]["num_trials"] = args.trials
    if args.rollouts is not None:
        config["study"]["rollouts_per_trial"] = args.rollouts
        config["study"]["evaluation_batch_size"] = min(
            config["study"]["evaluation_batch_size"], args.rollouts
        )
    _validate_config(config)
    evaluator = args.evaluator.expanduser().resolve()
    output_dir = _resolved_under_script(config["study"]["output_dir"])
    if args.evaluate_once:
        evaluate_once(
            config,
            evaluator,
            int(config["study"]["rollouts_per_trial"]),
        )
    else:
        output_dir.mkdir(parents=True, exist_ok=True)
        run_study(config, evaluator, output_dir)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
