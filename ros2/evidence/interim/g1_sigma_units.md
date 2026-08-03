# G1 — σ's units, resolved

**Correction to the interim block's record.** It reported: *"σ is not in
metres … `measurement_variance: 1.0` is a **unitless** knob."* That is half
right, and the half that is wrong changes what the fix is.

## σ has always been in metres. The VALUE was never set.

The tracker's filter is textbook linear Kalman
(`include/obstacle_detector/utilities/kalman.h`, `tracked_circle_obstacle.h`):

```
state q = [position, rate]        A(0,1) = T
C = [1 0]                          measurement y = a circle-centre coordinate, in METRES
R(0,0) = s_measurement_variance_   -> units m^2
P = A P A' + Q ; K = P C' (C P C' + R)^-1 ; P = (I - K C) P
```

`C` is dimensionless and `y` is metres, so `R`, `Q` and `P` are all m², and
`SurfaceSigma() = sqrt(max(var_x,var_y) + var_r)` is metres. There is no
missing scale factor anywhere in the chain, and there never was.

What is wrong is the number. `measurement_variance: 1.0` states that the
LiDAR measures an obstacle centre with a **one-metre standard deviation**.
P-2 seeds `P(0,0) = R` (two-point initiation), so every track is born with
σ = 1.0 m and relaxes to σ ≈ 0.58 m — exactly the p50 the interim block
measured. `k_σ` then absorbs the difference between that and the real error,
which is why neither the derived 0.287 nor H-8's 2.748 is a sigma multiplier.

The reason nothing caught it is worth recording: **1.0 m² is a perfectly
well-typed variance.** No unit check, no dimensional analysis and no type
system can distinguish "a variance in m²" from "a variance in m² that is half a
million times too large". Only data can.

## What `measurement_variance` should actually be

R is the variance of the **measurement noise** — the random scatter of the
extractor's circle-centre estimate. It is explicitly **not** the systematic
circle-fit bias (`e_c = −0.278·r`, `e_r = +0.084·r`), which is deterministic,
is already covered by `fixed_inflation = 0.051 m`, and would be
double-counted if folded in here.

New harness `test/measure_measurement_variance.py` derives it from data: on a
static scene it clusters `/raw_obstacles` detections per physical target and
reports the scatter about each target's mean, so the mean absorbs the bias by
construction. `phase4_obstacles_dump.py` now records `/raw_obstacles` — the
KF's actual measurements — so this needs no extra robot time.

## The sim number, and why it is not the shipped number

On `s1_surveyed` (three surveyed static cylinders, 300 detections each):

```
 target     n    mean x    mean y  sd x mm  sd y mm  sd r mm
      0   300     2.196    -2.203      1.4      0.5      1.2
      1   300     0.787     0.782      0.1      0.1      0.1
      2   300    -1.486     1.502      0.5      2.9      2.5

pooled position variance  1.775e-06 m^2  (1-sigma 1.3 mm)
pooled radius   variance  2.608e-06 m^2  (1-sigma 1.6 mm)
```

The shipped `1.0` is **563 431x** that. But 1.3 mm is not a candidate value
either: the simulated Mid-360 is an analytic raycast with no range-noise model,
so its measurement scatter is a discretisation artefact, not a sensor property.
Setting R to 1.8e-06 m^2 would tell the tracker to believe every measurement
almost absolutely and to ignore its own process model.

**Both endpoints are wrong for shipping, and that is the point:** what ships is
not a number, it is the machinery that stops a wrong one being used silently.

For calibration on the real unit, the arithmetic to expect: a Mid-360's range
noise is ~2 cm, and a circle centre is a fit over many returns but also
inherits projection and odometry jitter, so an obstacle-centre sigma of
1-3 cm is the plausible band, i.e. R ~ 1e-4 to 1e-3 m^2. Worth noting without
claiming it: at sigma ~ 20 mm, H-8's `k_sigma = 2.748` buys 55 mm of
inflation, which sits right beside the Phase-4-calibrated
`fixed_inflation = 0.051 m`. That is consistent with 2.748 having been derived
on a branch where `measurement_variance` WAS set from data — a hypothesis for
5B to confirm or kill, not a result.

## Decision: measured and recorded, NOT shipped from sim data

`measurement_variance` is a **sensor** property. Deriving it from the
simulator's LiDAR noise model and shipping it would re-tune the tracker — R
sets how much every position innovation is trusted, so it moves tracked
positions, tracked velocities, T5's velocity RMSE and every containment
number downstream — on the strength of a number that describes a simulator
rather than a Mid-360. That is precisely the "looks authoritative, isn't"
failure this gap is about, committed in the other direction.

So: the sim value is measured and recorded here as the order of magnitude and
as proof the measurement path works end to end; **5B block 1 sets the shipped
value from hardware**, and doing so is a joint recalibration with the P-2
tracker tuning, to be re-verified against S1/S2 containment afterwards.

## What ships instead: σ can no longer be used silently while it is wrong

1. **`calibrate_k_sigma.py` refuses.** If the pooled σ p50 exceeds
   `SIGMA_PLAUSIBLE_MAX = 0.25 m` it prints why and emits **no** `k_σ`,
   instead of printing an authoritative-looking number. A robot-day data drop
   therefore lands in a correct path or in a loud one, never in a plausible
   wrong one.
2. **`safety_obstacle_filter` warns at runtime.** `gating.h` gains
   `kSigmaPlausibleMax` and a `Stats` struct; `Apply()` counts
   `sigma_implausible` and `sigma_clamped` separately — the remedies are
   opposite (fix R, versus trust the cap) — and the node emits throttled
   warnings naming the offending σ. Four new unit tests, including one
   asserting that passing no `Stats` leaves behaviour byte-identical.
3. **The unit is written down where the value lives**: Appendix A, the 5B
   checklist and `obstacle_detector.yaml` all now say `m²` and say what 1.0
   asserts.

## Consequence for P-3

The interim block's headline P-3 result — "coverage 92.5 % → 99.9 % pooled,
S3 85.2 % → 99.9 %, with `k_σ = 0.287`" — was computed on σ ≈ 0.58 m. Once R
is set from real data σ will fall by orders of magnitude, and `k_σ` will have
to rise by the same factor to buy the same inflation. Whether the *shape* of
the remedy survives depends on whether σ still correlates with the residual
after rescaling; the sim correlation was `corr(F_req, σ) = +0.339`, positive
but weak, and rescaling does not change a correlation. **So the honest
statement is that the shape of the remedy is unchanged and its magnitude is
uncalibrated — and if hardware does not improve on that correlation, per-track
covariance is the wrong observable and the occlusion residual needs the
detector-level arc-truncation fix instead.**
