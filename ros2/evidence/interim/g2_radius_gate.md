# G2 — obstacles with r ≳ 0.40 m were silently dropped

**Status: hypothesis confirmed arithmetically and empirically; gate semantics
aligned (fork patch 0009); both drop paths made observable.**

## The hypothesis, and the check

The interim block proposed: `detectCircles()`'s `max_circle_radius` gate
applies to the *enlarged* radius, so with `radius_enlargement = 0.17` and the
newly-derived fit bias `e_r = +0.084·r` the effective true-radius cutoff is
`(0.60 − 0.17)/1.084 ≈ 0.397 m`.

Source (`src/obstacle_extractor.cpp`, before the patch):

```cpp
Circle circle(*segment);
circle.radius += p_radius_enlargement_;
if (circle.radius < p_max_circle_radius_) { circles_.push_back(circle); ... }
```

So the kept condition is `fit + 0.17 < 0.60`, i.e. **`fit < 0.43 m`**. It was
not adopted because it is tidy — it was checked against every row of the
210-configuration analytic-ray sweep (`circle_fit_sweep.csv`), which records
the measured fit (`r_meas`) and whether anything was detected:

| prediction | rows | agreement |
|---|---|---|
| `fit + 0.17 ≥ 0.60` ⇒ no detection | 17 | 17/17 |
| `fit + 0.17 < 0.60` ⇒ detection | 43 (r ≥ 0.4 subset) | 43/43 |

No exceptions. One further non-detection in the sweep (`r_gt = 0.10`,
`d = 4.0 m`, `arc = 0.5`) is a *different* mechanism — too few returns for
`min_group_points = 5` — and is correctly not explained by this rule.

## The cutoff is not a single radius

`r_meas + 0.17` from the sweep, for the two largest radii:

| r_gt | arc 1.00 | 0.90 | 0.75 | 0.60 | 0.50 |
|---|---|---|---|---|---|
| 0.40 (d = 2.0 m) | **0.622 DROP** | 0.563 | 0.519 | 0.475 | 0.447 |
| 0.50 (d = 2.0 m) | **DROP** | **DROP** | 0.565 | 0.511 | 0.473 |

Truncating the visible arc shortens the chord, shrinks the fit, and brings the
*same physical object* back under the cut. A 0.40 m cylinder at 2 m is
invisible when fully visible and visible when a neighbour occludes 10 % of it;
a 0.50 m cylinder needs 25 % occlusion. Range matters too, through
`sqrt(1 − (r/d)²)`: r = 0.40 is detected at 1.0 and 1.5 m and dropped at 2.0,
2.5 and 3.0 m.

That is the safety-relevant part. The failure is not "objects above X are
missing", which an operator could work around, but "objects near X flicker in
and out as the robot moves", which looks like a tracking problem and is not
one.

## What §9.6 said, and what the code measured

§9.6 documented the bound as `true_radius > 0.60`. Three separate mismatches:

1. The extractor's gate is on `fit + radius_enlargement`, not on the fit — so
   `max_circle_radius` really meant *"max radius minus radius_enlargement"*,
   and making the conservative safety inflation larger silently made the
   sensor blinder. That coupling is backwards and is the actual defect.
2. `true_radius` (published as `radius − radius_enlargement`) is the *fitted*
   radius, which over-estimates a fully visible cylinder by `+8.4 %·r`. It is
   the best estimate available, but it is not the true radius.
3. `safety_obstacle_filter`'s own `true_radius > 0.60` gate was therefore
   **effectively unreachable**: the extractor could never emit a
   `true_radius ≥ 0.43`. (Post-patch it is *nearly* redundant rather than
   dead — see the observability log, where it fired once on a `true_radius`
   of 0.602 m. The extractor bounds its own **fit**; the value the filter
   sees has been through the tracker's KF, which can carry the radius a hair
   past that bound. So the gate does catch something the extractor cannot:
   a KF-drifted radius. Predicting it dead was half wrong, and the runtime
   counter is what showed it.)

## Decision

**Align the semantics** (fork patch 0009), rather than raise the threshold or
document the accidental one:

```cpp
Circle circle(*segment);
if (circle.radius < p_max_circle_radius_) {      // gate on the FIT
  circle.radius += p_radius_enlargement_;        // then inflate
  ...
} else { ++dropped_large_circles_; RCLCPP_WARN_THROTTLE(...); }
```

Rationale: the plausibility test ("this arc is too big to be an obstacle") and
the safety margin ("report obstacles slightly larger than measured") are
different ideas, and mixing them made one a function of the other. Downstream
is untouched — `circle.radius` still carries the enlargement when pushed, so
`mergeCircles()` and `true_radius = radius − enlargement` behave exactly as
before for every circle that was already being kept. The change can only
*admit* circles that were previously dropped.

`mergeCircles()`'s own `max_circle_radius` test is left arithmetically alone:
its quantity is a deliberate over-bound (`span + max(parent radii)`), not a
fit, and refusing a merge leaves both parents alive rather than losing an
obstacle. It is counted and logged all the same.

## Verified after the patch

Same harness, patched extractor, full arc, d = 2 and 3 m:

| r_gt | d = 2.0 m | d = 3.0 m |
|---|---|---|
| 0.30 | fit 0.3273 keep | 0.3199 keep |
| 0.35 | 0.3717 keep | 0.3771 keep |
| 0.40 | **0.4407 keep** (was DROPPED) | **0.4341 keep** (was DROPPED) |
| 0.45 | 0.4786 keep | 0.4910 keep |
| 0.50 | 0.5314 keep | 0.5477 keep |
| 0.55 | 0.5796 keep | DROP |
| 0.60 | DROP | DROP |
| 0.65 | DROP | DROP |

The last kept fit is 0.5796 m, just under `max_circle_radius = 0.60`, and the
first drop is where the fit would exceed it. The gate now measures the
quantity the parameter is named after, and the empirical true-radius limit is
**0.55 m**, against the predicted `0.60/1.084 = 0.553 m`.

## The resulting real-world sensing limit

**True radius ≲ 0.55 m** (`0.60 / 1.084`, fully visible; partial occlusion only
extends it). In physical terms, this pipeline can see:

* people standing or walking (r ≈ 0.20–0.30 m) — comfortably;
* seated people, office chairs, bins, traffic cones, crates up to ~1.1 m
  across — yes;
* structural pillars, pallet stacks, vehicles, and any wall — **no**, and by
  design: they are not circles, and the gate exists to stop the extractor
  fitting a fictitious 3-metre circle to a wall arc and handing DPCBF a
  constraint that swallows the arena.

The last row is a genuine limitation of the circle model, not of the
threshold, and §9.7's ground-segmentation/Phase-7 work is where a non-circular
obstacle representation would belong.

## Reconciliation with the prop bound

The interim block gave the operator `r ≥ 0.30 m but not above ≈ 0.32 m`. That
bound is **withdrawn**: it was a consequence of the accidental cut at 0.397 m
with a safety margin for the arc dependence. With patch 0009 the constraint on
props is only `r ≥ 0.30 m` (to exercise the radius-dependent bias) and
`r ≤ 0.55 m`. `phase5b_checklists.md` block 0 is updated.

## Regression: what 0009 moved, and what it did not

Full `phase4_ab_run.sh` re-run after the patch, against the interim block's
pre-patch numbers:

| | pre-0009 | post-0009 |
|---|---|---|
| containment F = 0, s1_static | 100.000 % | **100.000 %** |
| containment F = 0, s2_cross_05 | 100.000 % | **100.000 %** |
| containment F = 0, s2_cross_08 | 100.000 % | **100.000 %** |
| containment F = 0, s3_swarm | 67.195 % | 70.895 % |
| containment F = 0, s4_occlusion | 13.242 % | 13.242 % |
| containment F = 50 mm, pooled | 92.254 % | 92.900 % |
| §17.3 offline A/B, S1 | 0.9565 | 0.9565 |
| §17.3 offline A/B, S2(0.5) / S2(0.8) | 1.0000 / 0.9978 | 1.0000 / 0.9978 |
| §17.3 offline A/B, S3 | 0.8235 | 0.8265 |
| §17.3 offline A/B, S4 | 0.9456 | 0.9456 |

**S1 and S2 are unchanged to the last recorded digit** — the bar. S3 improves
slightly in both containment and A/B ratio, which is the predicted direction:
the patch can only *admit* circles, and in a swarm some merged arcs that used
to exceed the old 0.43 m cut now survive. S4 is untouched (its blocker is
r = 0.30 m, far from either threshold). Nothing regressed.

## Observability

A silent drop is the defect regardless of where the threshold sits, so both
paths now report:

* `obstacle_extractor`: `dropped_large_circles_` / `refused_large_merges_`,
  each with a throttled `WARN` naming the offending radius, the threshold and
  the running count (patch 0009).
* `safety_obstacle_filter`: `Stats{stale_messages, dropped_large_radius,
  sigma_clamped, sigma_implausible, ...}` filled by `Apply()` and reported by
  the node with throttled warnings; four new unit tests cover the counters,
  including that passing no `Stats` leaves behaviour byte-identical.

Both fire live — `g2_drop_warnings.log`, captured from the post-patch sweep:

```
[safety_obstacle_filter]: dropped 1 circle(s) with true_radius > max_circle_radius 0.60 m (largest 0.602 m) — INVISIBLE to DPCBF
[obstacle_extractor]: dropped a circle of fitted radius 0.604 m (max_circle_radius 0.600 m) — an obstacle this large is INVISIBLE downstream; 1 dropped since start
[obstacle_extractor]: dropped a circle of fitted radius 0.720 m (max_circle_radius 0.600 m) — ...; 22 dropped since start
```

Before this block that entire sequence was silent.
