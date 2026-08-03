# Circle-fit bias: radius × range × visible-arc extent

Interim block (between Phases 5A and 5B). Harness:
`g1_perception_bringup/test/circle_fit_sweep.py`. Raw data:
`circle_fit_sweep.csv` (210 rows, 193 detections).

## What was measured and why it is trustworthy

Phase 3 measured the extractor's circle-fit error at **one** radius (0.15 m).
Phase 4 found r=0.30 gives ~83 mm centre bias and ~33 mm radius over-estimate,
and sized `fixed_inflation = 0.051 m` to cover it — but could not say whether
the systematic was a constant, a function of radius, or a function of range.

This sweep answers that. It publishes **analytically ray-cast** `LaserScan`
messages for a single cylinder of known radius at known range directly into the
real `obstacle_extractor` (same `obstacle_detector.yaml` the stack ships) and
reads `/raw_obstacles` back. Odometry, projection and tracking are all out of
the loop, ground truth is exact by construction, and the scan geometry
(`angle_increment` 0.0058, `range_min/max` 0.3/5.0) is §9.4's verbatim.

Sanity check against the numbers the project already had, at full visible arc:

| | this sweep | previously measured |
|---|---|---|
| r=0.15, centre error | 34–44 mm | Phase 3 T4: 40.9 / 38.4 / 40.1 mm at 1/2/3 m |
| r=0.15, radius error | +8.5…+16.9 mm | Phase 3 T4: +16.7 / +19.7 / +16.0 mm |
| r=0.30, centre error | 76–103 mm | Phase 4: 83 mm |
| r=0.30, radius error | +14…+35 mm | Phase 4: +33 mm |

Same family, independently produced. One correction to the record: Phase 4
reported the centre bias as a magnitude. It is **signed, and the sign matters** —
the reported centre is pulled **towards the sensor**, never away.

## The functional form

With `circles_from_visibles`, the extractor fits a line to the visible arc and
then builds the circle as the circumcircle of the equilateral triangle on that
chord (`utilities/circle.h`): `radius = (√3/3)·L`, `centre = midpoint −
(radius/2)·normal`. Both outputs therefore scale with the chord, and the chord
scales with the true radius. That is what the data shows.

**Full visible arc, fit through the origin over r ∈ [0.10, 0.40], d ∈ [1, 4] m:**

```
e_r  =  +0.084 · r_gt      residual rms  5.2 mm
e_c  =  -0.278 · r_gt      residual rms  6.0 mm      (radial, − = towards sensor)
```

Both biases are **proportional to the true radius and independent of range**
over 1–4 m — mean `e_c/r` per range: −0.304, −0.279, −0.278, −0.254, −0.258,
−0.280 at 1.0/1.5/2.0/2.5/3.0/4.0 m. The residual scatter is arc-endpoint
quantisation: the tangent-most returns land on discrete beams, so the chord
length steps.

The safety-relevant combination is the inflation a pair needs for containment
(`phase4_containment.py`'s `F_req`, velocity term zero on a static target):

```
F_req = |e_c| − e_r  ≈  0.20 · r_gt          (r ≥ min_radius; measured +0.223·r)
```

The two errors partly cancel: the circle is reported too close AND too big, so
the near surface is conservative and only the far side is short.

## Arc truncation — where a fixed term stops working

`arc` is the fraction of the cylinder's angular silhouette that is visible; the
loss is one-sided, the shape a neighbouring obstacle actually produces.

| arc | e_r / r | e_c / r | F_req / r (r ≥ 0.20) | max F_req |
|---|---|---|---|---|
| 1.00 | **+0.078** | −0.276 | +0.223 | 89 mm |
| 0.90 | −0.034 | −0.366 | +0.465 | 172 mm |
| 0.75 | −0.164 | −0.439 | +0.684 | 236 mm |
| 0.60 | −0.284 | −0.480 | +0.847 | 286 mm |
| 0.50 | −0.367 | −0.493 | +0.956 | 299 mm |

Required inflation in mm, `F_req ≈ k(arc)·r`:

| r_gt | arc 1.00 | 0.90 | 0.75 | 0.60 | 0.50 |
|---|---|---|---|---|---|
| 0.15 | 33 | 70 | 103 | 127 | 143 |
| 0.20 | 45 | 93 | 137 | 169 | 191 |
| 0.25 | 56 | 116 | 171 | 212 | 239 |
| 0.30 | 67 | 140 | 205 | 254 | 287 |
| 0.40 | 89 | 186 | 273 | 339 | 382 |

Two things follow.

1. **The radius error changes sign at ~10 % occlusion.** A fully visible
   cylinder is reported slightly too large (safe); a partly occluded one is
   reported too small — up to −37 % of its radius at half visibility — while
   the centre error keeps growing. Both move the same way, so `F_req` rises
   steeply. This is a quantitative account of the residual Phase 4 recorded and
   could not cover: S3's 15 % of pairs and S4's crosser coast are the
   arc-truncation regime.

2. **A fixed inflation is the wrong shape.** The requirement is a *product* of
   radius and occlusion. `fixed_inflation = 0.051 m` covers r=0.30 only while
   it is fully visible (67 mm needed — already marginal), and covers nothing at
   r=0.30 with 25 % occluded (205 mm). This is the strongest evidence yet for
   the §9.6 σ term: occlusion is exactly the condition under which the tracker's
   own posterior variance grows, which is the premise `calibrate_k_sigma.py`
   tests. It is **not** evidence for any particular `k_σ` — see below.

## A drop-out that has to be fixed before robot day

17 of the 210 configurations produced **no detection at all**, and they are not
random: they are the large radii.

| r_gt | ranges with no detection (full arc) |
|---|---|
| 0.40 | 2.0, 2.5, 3.0 m |
| 0.50 | 1.0, 1.5, 2.0, 2.5, 3.0, 4.0 m (and most truncated arcs) |

Mechanism: `detectCircles()` keeps a circle only if
`r_fit + radius_enlargement < max_circle_radius`, i.e.
`1.084·r + 0.17 < 0.60` → **r < 0.40 m**. Above that the obstacle is silently
discarded — no warning, no partial detection. `safety_obstacle_filter` then
applies its own `max_circle_radius: 0.60` gate to `true_radius`, a second
identical trap.

Consequences, both actionable now:

* 5B's checklist asks for a prop with **r ≥ 0.30 m**. At r=0.30 the margin to
  the cut is 8.5 cm of fitted radius; at r=0.35 it is gone. **Either keep the
  large prop at r ≤ 0.32 m, or raise `max_circle_radius` in BOTH
  `obstacle_detector.yaml` and `safety_obstacle_filter.yaml` before the
  session.** A prop that is invisible for geometric reasons would read on the
  day as a perception failure.
* Perversely, truncation can *restore* detection (r=0.50 is detected at
  arc ≤ 0.75 because the shorter chord fits under the cut) — so the failure is
  intermittent as the robot moves, which is the worst way for it to present.

Not changed in this block: retuning `max_circle_radius` trades against its
actual job (rejecting spurious wall arcs, §9.6 step 1), and that trade needs
the hardware wall data 5B captures. Recorded as a checklist item instead.

## What this does NOT establish

* No correction has been implemented. A tangent-pair estimator (fit the circle
  tangent to the two extreme rays instead of circumscribing the chord) is the
  obvious candidate and is derivable from what the extractor already has, but it
  under-estimates under truncation — the unsafe direction — so it needs the
  hardware arc statistics before it can ship. Deferred with the reason.
* Every number here is from an analytic scan with no range noise, no incidence
  drop-out and no ground returns. It isolates the fit; it does not predict the
  hardware. 5B's T4-hardware run (`T4_BAG`/`T4_LAYOUT`) measures the same
  quantities on real props and is the check.
