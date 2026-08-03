# P-3 σ path — wiring validation on sim data

Interim block (between Phases 5A and 5B). This is a **rehearsal, not a
calibration**: every σ below comes from sim covariances, so no `k_σ` derived
here may ship. What it establishes is that the whole path exists and carries
real numbers, and — unexpectedly — that the σ it carries is not in metres.

## The path

```
obstacle_tracker (patch 0007)   ->  CircleObstacle.covariance = [var_x, var_y, var_r]
                                    = P(0,0) of the three per-axis KFs
safety_obstacle_filter          ->  SurfaceSigma() = sqrt(max(var_x,var_y) + var_r)
                                    radius += k_sigma * min(sigma, sigma_max)
                                    (use_covariance: false — SHIPPED DISABLED)
phase4_obstacles_dump.py        ->  records `cov` per circle
calibrate_k_sigma.py            ->  (F_req, sigma) pairs -> coverage, k_sigma
```

Covariance is only meaningful because P-2 (Phase 3) initialises
`P = diag(R, 2R/dt²)` instead of upstream's identity. Producers that are not
the tracker — the oracle source, hand-built test messages — leave it zero, and
`SurfaceSigma` then returns 0, so the term self-disables rather than mis-fires
(unit test `SigmaSelfDisablesOnNonTrackerProducers`).

## Containment is unchanged where it was already good

Re-ran `phase4_ab_run.sh` end to end on the regenerated fixtures. §17.3 A/B
reproduces Phase 4 exactly — S3 perf ratio **0.8235**, the same figure the
Phase-4 §21 entry records — so the regenerated bags and the P-3 build are a
faithful re-run, not a new baseline.

Containment with the shipped `fixed_inflation = 0.051 m`, σ term OFF:

| scenario | pairs | coverage |
|---|---|---|
| s1_static | 598 | 100.000 % |
| s2_cross_05 | 89 | 100.000 % |
| s2_cross_08 | 56 | 100.000 % |
| s3_swarm | 884 | 85.181 % |
| s4_occlusion | 219 | 96.804 % |

**S1/S2 unchanged at 100 %** — the evidence bar for this block. S3/S4 are the
Phase-4 residual, unchanged, as expected.

## With the σ term on (offline, on the same pairs)

```
corr(F_req, sigma) = +0.339
fixed = 51 mm, k_sigma covering 99.9 % of pooled pairs = 0.287

  coverage fixed-only            92.524 %
  coverage fixed + 0.287*sigma   99.946 %
  coverage fixed + 2.748*sigma  100.000 %   (old-branch H-8 placeholder)

per scenario, fixed-only -> fixed + 0.287*sigma
  s1_static     100.000 -> 100.000
  s2_cross_05   100.000 -> 100.000
  s2_cross_08   100.000 -> 100.000
  s3_swarm       85.181 ->  99.887
  s4_occlusion   96.804 -> 100.000
```

The term does what it was hypothesised to do: it leaves the easy scenarios
alone and rescues exactly the occlusion/density regime a fixed term cannot
reach. That is a real result about the *shape* of the remedy.

## The finding that actually matters: σ is not in metres

σ p50 is **583 mm**, and 8.1 m at the S3 tail. Those are not plausible position
uncertainties for a 10 Hz lidar tracker looking at a 3 m arena. They are
correct arithmetic on a covariance that was never given physical units:
`obstacle_detector.yaml` sets `measurement_variance: 1.0`, a **unitless tuning
knob**, and P-2 seeds `P(0,0) = measurement_variance`. So σ ≈ √(0.34) ≈ 0.58 is
a *relative* uncertainty, and `k_σ` silently absorbs the missing scale.

Consequences, all of which are 5B work:

1. **`k_σ = 0.287` is not comparable to the old branch's 2.748** and neither is
   a metre-scale multiplier. Quoting either as "the calibrated k_σ" would be
   wrong. Both are scale factors against whatever `measurement_variance`
   happens to be.
2. **The fix is upstream of `k_σ`:** set `measurement_variance` to the measured
   range-noise variance in m² (5B block 1 gives it: the per-return range noise
   of the Mid-360 at 1–3 m). Then σ is metres, `k_σ` is a genuine sigma
   multiplier, and a value near 2–3 would be the expected answer.
3. Until then `use_covariance` stays **false** and `fixed_inflation = 0.051`
   stays the shipped behaviour. `sigma_max = 0.50 m` exists so that a diverged
   track cannot inflate without bound; with unitless σ that cap currently binds
   almost immediately, which is a further reason not to enable the term.

`corr(F_req, σ) = +0.339` is positive but weak — the premise holds
directionally, not strongly. Worth re-measuring on hardware before committing
to the σ term at all; if the correlation does not improve with physically
scaled covariances, the honest conclusion is that per-track covariance is the
wrong observable and the occlusion residual needs a detector-level fix (the
arc-truncation model in `circle_fit_bias.md`) rather than an inflation term.

## Reproduce

```bash
ros2/src/g1_perception/g1_perception_bringup/test/phase4_ab_run.sh /tmp/interim_ab
/usr/bin/python3 .../calibrate_k_sigma.py \
    s1_static=/tmp/interim_ab/s1_static.jsonl ... --fixed 0.051
```

On robot day the second command is unchanged; only the dumps come from
`phase4_obstacles_dump.py` run against the hardware session.
