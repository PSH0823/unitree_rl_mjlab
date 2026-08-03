# Walking closed-loop A/B — the two safety numbers (2026-08-02)

Deferred six times, because until this block the robot could not be made to
walk headlessly. It can now: `test/walk_ab_run.sh` is one command that brings
g1_ctrl's FSM up through Passive → FixStand → RLBase under a scripted joystick
profile, lowers the support band onto the running policy, and drives the
policy through ~110 s of locomotion with the full perception stack live and
DPCBF in the 1 kHz seam — once per DPCBF mode on an identical seeded field.

## What is being measured, and what it is not

`phase4_ab_run.sh` (Phase 4, kept and still run) replays the S1–S4 fixture
bags against a robot **pinned at (0,0,0)**. It scores command tracking. A
robot that never moves has no collision rate and no clearance, which is why
those two numbers have never existed.

This harness measures, at 100 Hz from the simulator's own ground truth:

* **clearance(t)** = `min_i(|p_rob − p_i| − r_rob − r_i)` with `r_rob = 0.30`
  from `dpcbf_config.yaml`. Negative means the robot's safety disc overlaps an
  obstacle's — a **DPCBF margin violation**, the constraint DPCBF exists to
  enforce.
* **fall** (`tilt > 0.6 rad` or `pelvis z < 0.5 m`), reported separately and
  ending the scoring window.

The two are deliberately not merged into one "collision rate". A margin
violation is not necessarily body contact — `r_rob` is a disc about the
pelvis and the limbs reach outside it. A body contact against a 1.5 m
cylinder (`collision_enabled: true`, so contact is real contact) shows up as
a fall. One number would have hidden which happened.

Only obstacles within `p_max` (3.0 m) are scored: DPCBF culls beyond that, so
a filter cannot be blamed for what it was never given.

**This is a distributional comparison, not a paired one, and one run per cell
is a sample rather than a measurement.** The two arms start from identical
seeds and run an identical command profile, but their trajectories diverge the
moment the two filters disagree, so they do not meet the same obstacles at the
same times. Phase 4's offline harness avoided that by pinning the robot; the
price of a moving robot is that pairing is gone.

Worse, the loop is chaotic and live runs are wall-clock nondeterministic
(H-10, established in Phase 4). **W3 oracle was run twice on the identical
seeded field and the two runs disagree qualitatively** — run A walked the full
110 s with 7 margin violations and 2.79 obstacles inside `p_max` on average;
run B walked into a corner of the 8×8 arena, met 10.24 obstacles inside
`p_max`, took 9 violations and fell at 53.5 s. Both are honest samples of the
same configuration. Every number below is therefore a single observation
unless a repeat count is stated, and the dense-field cells (W3, W4) should be
read with that in mind. `repeats/` holds two further W4 pairs run for exactly
this reason — to test whether the headline oracle-vs-estimated asymmetry
survives repetition — and their outcome is at the end of this file.

## The fields

These are **not** the S1–S4 fixture bags and must not be quoted as them. The
fixtures are bag replays with the simulator out of the loop; a live closed
loop needs a live obstacle source, which on this stack is the DPCBF
`DynamicObstacleManager` seeded arena. They are matched on the property under
test — count, radius range, speed range — and W4 is the Phase-4 live arena
verbatim, seed and all.

| | count | radii | speeds | arena | seed | analogue |
|---|---|---|---|---|---|---|
| W1 | 6 | 0.20–0.30 | 0 | 8×8 | 20260802 | S1, sparse static |
| W2 | 6 | 0.20–0.30 | 0.5–0.8 | 8×8 | 20260802 | S2, sparse crossing |
| W3 | 20 | 0.20–0.30 | 0.2–0.8 | 8×8 | 20260801 | S3, swarm |
| W4 | 90 | 0.20–0.30 | 0–0.8 | 20×20 | 42 | Phase-4 live arena |

`dpcbf/` is byte-for-byte untouched: the per-field configs are copies in a
scratch run tree (`walk_scenarios.py`).

All rows below were taken on **one** code version, the pre-0009 extractor, so
the matrix is internally consistent. Patch 0009 can only *admit* circles that
were previously dropped, and the largest true radius in any of these fields is
0.30 m (fit ≈ 0.33 m, nowhere near the 0.43 m cut), so it does not move these
numbers except through merged arcs.

## Robot state reached, against the bar

The bar: "stable locomotion under a scripted command profile, long enough to
run the A/B scenarios; standing, twitching or being dragged is not walking."

**Met.** Pelvis 0.771–0.788 m against a 0.793 m standing height, maximum
|roll,pitch| **0.046 rad**, 110 s of continuous locomotion per run, 47–52 m of
path at 0.43–0.48 m/s, `bad_orientation` never firing in any oracle run.

## Results

See `ab/walk_ab_report.txt` for the generated table and `ab/*_metrics.json`
for the raw numbers.

### The headline

* **W1 (sparse, static) — PASS.** Zero margin violations in both arms, no
  falls. Estimated mode is *more* conservative than oracle (min clearance
  0.193 m vs 0.020 m) because the safety filter's inflation pushes DPCBF's
  constraints out. This is the class Phase 4 already passed with margin, and
  it still passes with the robot walking.
* **W4 (the Phase-4 90-obstacle arena).** First run: oracle 0 margin
  violations and no fall, min clearance 0.021 m over 110 s; estimated 5
  violations, min clearance −0.215 m, and a fall at 70.4 s that the trace
  shows to be a collision — clearance crosses zero at t = 69.83 s and the
  robot pitches (0.078 → 0.093 → 0.177 → 0.268 → 0.397 rad) and drops
  (z 0.773 → 0.640 m) over the next 0.3 s. It walked into a cylinder.
  **Two further pairs were then run, and they withdraw the "oracle is clean"
  half of that — see "What repetition did to the headline" below.** The
  collision itself stands; what does not stand is the baseline it was scored
  against.
* **W2 (sparse, crossing) — oracle not clean.** Oracle already takes 1 margin
  violation (min clearance −0.135 m); estimated takes 5 (−0.156 m). Neither
  arm falls. The estimated arm cannot be scored as a *perception* result
  against a baseline that itself violates — see below.
* **W3 (swarm) — no verdict.** Both oracle runs and the estimated run
  encountered densities from 2.8 to 10.2 obstacles inside `p_max` depending on
  where the policy wandered in an 8×8 box, and two of the three fell. The cell
  is too high-variance at one run each to support a comparison; it is reported,
  not concluded from. `W3_oracle_runA_transcribed.json` preserves run A's
  numbers, whose files were lost to an aborted re-run of mine.

### What repetition did to the headline

Two further W4 pairs were run on the identical seeded field
(`repeats/r2`, `repeats/r3`). All three pairs:

| run | oracle | estimated |
|---|---|---|
| matrix | 0 violations, no fall, min cl +0.021 m | 5 violations, min −0.215 m, **fell 70.4 s** (collision) |
| r2 | **11 violations**, no fall, min −0.067 m | **fell 37.9 s** (pre-window) |
| r3 | 2 violations, **fell 115.5 s**, min −0.217 m | **fell 35.3 s** (pre-window) |

**Two claims must be withdrawn and one survives.**

1. **WITHDRAWN: "oracle mode is clean on W4."** Across three runs of the same
   configuration the oracle arm took **0, 11 and 2** margin violations and
   fell once. The clean first run was luck. With exact ground truth, zero
   latency and zero association error, DPCBF + this policy is *not* reliably
   inside the margin in a dense field.
2. **WITHDRAWN: the margin-violation ordering on W4.** The oracle arm's own
   spread (0–11) brackets and exceeds the single scoreable estimated value
   (5). At n = 3 the counts cannot separate the arms, and quoting 0 vs 5 as
   an oracle-vs-estimated result would be quoting noise.
3. **SURVIVES: the fall ordering.** Estimated fell in **3 of 3** runs, at
   35.3, 37.9 and 70.4 s; oracle fell in **1 of 3**, and latest of all at
   115.5 s. That is consistent across every pair.

   With one caveat that keeps it from being conclusive: two of the three
   estimated falls (35.3 and 37.9 s) are before the first walking command and
   are the band-transfer class, which `diag/` shows a later band release
   largely removes. So the fall statistic is partly contaminated by this
   harness's bring-up and **must be re-run on the `34,6` release before it
   carries weight**.

### The delta metric IS reproducible, even though the safety counts are not

The same three W4 oracle runs give tracked→GT p50 **104.8 / 99.8 / 103.9 mm**
and p90 **321.7 / 293.4 / 280.9 mm** — a spread of ±3 % at p50 and ±7 % at p90,
over 27–32 k samples each. That is a different reliability class entirely from
the margin-violation counts (0 / 11 / 2 on the same runs).

The reason is sample size and what the statistic is: a percentile over ~30 000
per-obstacle comparisons averages over the whole run, whereas a violation
count is a handful of rare events whose occurrence depends on exactly where a
chaotic trajectory happened to go. **So the two halves of this block's walking
result have very different weight**, and they should be quoted differently:

* **The shadow deltas and everything resting on them — including the
  rig-vs-density verdict on S3 — are solid.** They reproduce across runs.
* **The collision rate and clearance distribution are a first sample.** They
  need repeats before they can gate anything.

**What this block therefore establishes about W4** is that the estimated arm
is markedly less stable than the oracle arm, that at least one of its failures
is a genuine collision, and that **the baseline is not clean either** — not a
clean oracle-vs-estimated separation. The remaining cells (W1 both arms clean;
W2 oracle 1, estimated 5) are single observations and inherit the same
uncertainty.

**Ordering across fields**, stated at the strength the data supports: W1 0 → 0,
W2 1 → 5, W4 (0, 11, 2) → (5, –, –). Estimated is never *better* than oracle in
any run, but only the W4 fall counts and the W2 pair point in a consistent
direction, and neither is repeated enough to be a gate.

#### Two of the falls were my bring-up, not perception — and one was not

W3 and W4 estimated both first fell at t ≈ 30 s, which is 2 s after the
support band finished lowering (release 24 s, ramp 4 s) and 10 s *before* the
first walking command. That is a load-transfer transient, not a driving
failure, and it needed a discriminator rather than an assumption.

`diag/` holds it: the same W3 estimated configuration with the band lowered at
34 s over 6 s instead of 24 s over 4 s. It then **walked 73 s** (50 → 123 s)
with 4 margin violations and fell at **122.95 s** — deep into locomotion, not
at the transfer. So the early falls were the procedure: DPCBF is already
commanding evasive velocity when the policy takes the robot's weight, and in
estimated mode the inflated radii make that command larger.

The W4 estimated fall at **70.4 s** is *not* in that class — it is 42 s after
the band is gone, and the trace shows the collision directly (clearance
crosses zero at 69.83 s, then the pitch and drop). The distinction matters:
one failure mode is an artefact of how this harness stands the robot up, the
other is the thing the harness exists to measure.

**The committed default is left at `24,4`, matching the numbers above**, so
the shipped harness reproduces what is reported here. `WALK_BAND_RELEASE=34,6`
is the better bring-up and is documented; re-running the matrix on it is a
named follow-up, not a silent change.

## The oracle arm is not clean, and that is its own finding

In W2 and W3 the oracle arm — which sees exact ground truth, zero latency,
zero association error — still lets the pelvis disc overlap an obstacle disc
by up to 0.24 m. That is not a perception result. **DPCBF's guarantee is on
the commanded velocity; the realised base motion is the RL policy's, and its
velocity-tracking error is not modelled anywhere in the chain.** A filtered
command the policy under-tracks by a few tenths of a m/s for a few hundred
milliseconds is enough to cross a 0.30 m margin. Two further unmodelled
terms: `r_rob = 0.30` is a disc about the pelvis, while a walking G1's feet
and arms swing well outside it; and DPCBF's `s = 1.05` safety factor is
sized against command error, not tracking error.

This is a new, measurable, closed-loop finding that neither the offline A/B
nor the containment sweep could produce, and it is upstream of perception: no
amount of perception improvement fixes it.

## The re-priced shadow deltas, and the rig-vs-density verdict

`tracked → nearest-GT` distance per tracked circle within `p_max`, capped at
0.5 m (Phase 4's NN cap). **p50 and p90 are comparable with Phase 4's; p99 is
cap-limited at 500 mm in this harness and is not.**

The rig control's own clearance and margin-violation numbers (6 violations,
min −0.236 m) appear in `walk_ab_report.txt` because the probe computes them
unconditionally, but they mean **nothing about safety**: the robot is hanging
from a band with no controller, drifting through the field under no one's
command. Only its **tracked→GT** row is a measurement. Read the rig row for
the deltas and ignore the rest of it.

A **rig control** was run to make the rig↔walking comparison a controlled one
rather than a comparison against a number recorded four phases ago: identical
field (90 obstacles, seed 42), identical code, identical probe, band never
lowered and no `g1_ctrl`, so the robot hangs limp exactly as it did when the
Phase-4 shadow deltas were taken (`ab/W4_rig*`; measured pelvis z 1.338 m,
confirming the hoist).

| condition | count | speeds | GT in p_max | p50 | p90 |
|---|---|---|---|---|---|
| **W4 rig control**, suspended, this code | 90 | 0–0.8 | 8.97 | **343** | **470** |
| W4 oracle, same arena, **walking** | 90 | 0–0.8 | 4.41 | **105** | **322** |
| W3 oracle, swarm, walking (run A) | 20 | 0.2–0.8 | 2.79 | 120 | 341 |
| W2 oracle, sparse moving, walking | 6 | 0.5–0.8 | 0.87 | 116 | 311 |
| W1 oracle, sparse static, walking | 6 | 0 | 0.72 | 80 | 209 |
| *Phase-4 record, same field, suspended* | 90 | 0–0.8 | — | *94* | *186* |

Decomposition within the walking runs (p50 / p90, mm):

| term | Δp50 | Δp90 |
|---|---|---|
| **obstacle motion** (W1 → W2, same 6 obstacles, same seed) | **+36** | **+102** |
| density 6 → 20 (W2 → W3, both moving) | +4 | +29 |
| density 20 → 90 (W3 → W4) | −15 | −19 |
| **rig removed** (W4 rig control → W4 walking, same field, same code) | **−238** | **−148** |

### The suspension rig is a large artifact — larger than Phase 4 recorded

On identical code and field, the suspended robot's tracked→GT p50 is **343 mm
against walking's 105 mm**: the rig is 3.3× worse. That is the opposite sign
to what the Phase-4 record suggests, and it is the controlled measurement, so
it is the one to use.

**Phase 4's recorded 94 / 186 mm was not reproduced and I cannot reconcile
it.** Two candidate causes, neither verifiable now: the numbers came from a
different measurement path (the adapter's internal shadow accumulator at query
time, against this ROS-level probe), and the limp robot's attitude is an
uncontrolled variable — this control saw |roll,pitch| up to **0.456 rad**
(26°), and a 26°-tilted LiDAR projected into a 0.15–1.60 m band above
`base_footprint` slices the cylinders at heights the circle fit was never
meant for. Phase 4 recorded the rig's swing and free yaw but not its tilt. The
honest position: the two rig numbers disagree by 3.6×, this one is
internally controlled, and **the Phase-4 live-arena figures — the 0.686
est/oracle ratio and the 94/186 mm deltas — should be treated as measured on
an uncontrolled rig and superseded by the walking numbers, not averaged with
them.**

### Verdict on S3's 0.8235: a real perception limit, not an evaluation artifact

1. **By construction, S3 contains no rig at all.** The S1–S4 fixture bags were
   recorded with a constant grounded `qpos`
   (`test_fixtures/scenarios/make_scenario_scene.py::grounded_qpos` — pelvis
   z = 0.793, identity quaternion, never updated). The rig existed only in the
   Phase-4 *live-arena* session, which is where the 0.686 ratio and the
   94/186 mm deltas came from. S3's 0.8235 was measured with a perfectly
   stationary sensor: there was never a rig artifact in it to remove. **This
   leg alone settles the question, and it does not depend on any measurement
   taken this block.**
2. **The dominant error term among the things that *are* in S3 is obstacle
   motion.** Holding count and seed fixed and only starting the obstacles
   moving costs +36 mm at p50 and +102 mm at p90 — more than the entire
   density span from 6 to 90 obstacles. That is the tracker's
   coast-and-re-acquire behaviour and the extractor's arc truncation on moving
   targets: exactly the Q-2 residual class Phase 4 identified and could not
   cover with a fixed inflation.
3. **S3's number is a lower bound.** Its sensor is perfectly still; a walking
   sensor is 2–4× worse than a static one on the same scene class.

**So: S3/S4 is a real perception limit and belongs in Phase 7 remediation.**
The rig was a real and large artifact, but it was never in S3 — it was in the
Phase-4 *live-arena* numbers, and those are the ones this block supersedes.

## Known limitations of this harness

* Distributional, not paired (above).
* p99 of the tracked→GT distribution is truncated by the 0.5 m association
  cap; only p50/p90 should be compared with Phase 4.
* The scoring window covers only the fraction of time an obstacle is inside
  `p_max` (28 % in W1, 77 % in W4); everything else is `NaN` and excluded.
* The trace does not record the per-obstacle association at the moment of
  contact, so the W4 collision is identified as "walked into a cylinder"
  rather than attributed to a specific tracking failure. Carried as a
  follow-up.


## Visual evidence (carried since Phase 1)

`walk_overlay.png` — five panels from a live W3 estimated-mode session,
rendered **offscreen** (matplotlib Agg: no display, no RViz, no GL) from 7454
frames over sim t = 50 → 136 s. Four snapshots plus the whole-run base
trajectory, all in the odom frame: `/scan` (grey), `/tracked_obstacles` (red,
solid), `/sim/gt_obstacles` (blue, dashed), robot and its `p_max` horizon
(green). Produced with `walk_overlay_run.sh`.

It shows more than the interactive screenshot would have, because tracked and
GT are overlaid per obstacle rather than summarised as a percentile — and two
things are visible in it that are worth stating:

* **At t = 50 s the agreement is good**: 14 tracked circles sit on their GT
  counterparts, and the red circles are visibly *larger* than the blue ones,
  which is `fixed_inflation` + the velocity term doing exactly what §9.6 says.
* **Recall varies enormously through the run** — 14 tracked at t = 50 s, then
  1, 6 and 1 in the later panels, against a constant 20 GT with several inside
  `p_max` each time. The long straight grey runs along the arena edges are the
  boundary geometry, correctly *not* turned into circles by the
  `max_circle_radius` gate. The snapshots are qualitative and a single frame
  proves nothing on its own, but they are consistent with the swarm-density
  limit the numbers above price, and they are a reminder that the delta
  percentiles are computed only over circles that were detected at all —
  **a missed obstacle contributes no error to the tracked→GT distribution.**
  That asymmetry is worth a dedicated recall metric; there isn't one yet.
