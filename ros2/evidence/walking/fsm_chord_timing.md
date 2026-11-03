# Why the scripted FSM chords never fired (workstream B, 2026-08-02)

The `ScriptedJoystick` buttons added in the interim block compile, are packed
into `wireless_remote` correctly, and arrive at `g1_ctrl` correctly. The FSM
still never left `State_Passive`. An unreceived button and a rejected
transition look identical from outside, so the question was settled with an
instrument rather than by inspection: `simulate/build_ros2/fsm_button_probe`
runs the FSM's own decode (`unitree::robot::g1::subscription::LowState`) and
its own compiled transition DSL (`deploy/include/unitree_joystick_dsl.hpp`,
fed the condition strings out of `deploy/robots/g1/config/config.yaml`) at
1 kHz against a live `rt/lowstate`. It touches nothing; `deploy/` is read, not
modified.

## The mechanism

`deploy/robots/g1/config/config.yaml` transitions:

```yaml
Passive:  { transitions: { FixStand: "LT + up.on_pressed" } }
FixStand: { transitions: { Velocity: "RT + A.on_pressed" } }
```

The DSL parses `LT + up.on_pressed` as `AND(LT.pressed, up.on_pressed)` —
both operands must be true **on the same tick**.

* `up` is a `Button<int>`: `up.on_pressed` is true for exactly **one** tick,
  the first one after the bit changes.
* `LT` is an `Axis`. `g1_sub.h::LowState::update()` feeds it the raw L2 bit,
  and `Axis::operator()` low-passes with `smooth = 0.03` before comparing
  against `threshold = 0.5`. Starting from 0, that needs `1 - 0.97ⁿ > 0.5`,
  i.e. **n = 23 ticks**.

Setting `LT.smooth = 1.0` on the *sender's* joystick (the interim block's fix)
does not help: the sender's `pressed` flag is what gets packed into the
wireless_remote bit, and it is the **receiver's** `Axis` that lags.

So pressing `L2,up` at one profile breakpoint can never satisfy the AND: on
the tick where `up.on_pressed` fires, `LT.pressed` is still false, and by the
time `LT.pressed` goes true, `up.on_pressed` is long gone.

## The measurement

Profile: `15.0 → L2,up` held for 0.6 s (`fsm_probe_simultaneous.log`,
`fsm_probe_simultaneous_window.csv`).

```
 t          tick    LT_p  LT_val     up_p  up_op   Passive->FixStand
 7.705      15000    0    0.03        1     1       0     <- the only edge
 7.706      15000    0    0.0591      1     0       0
 ...
 7.72705    15022    1    0.503694    1     0       0     <- 22 ticks later
```

Both keys are received (`LT` and `up` each report 601 pressed ticks and one
`on_pressed` edge; the receiver-side axis peaks at 0.999999). **All seven**
transition predicates in the config fired **zero** times.

Staggering the chord — axis half first, button edge ≥25 ms later
(`fsm_probe_staggered.log`) — fires both required predicates exactly once:

```
  Passive -> FixStand    <- LT + up.on_pressed   fired 1 ticks  first @ 8.187 s
  FixStand -> Velocity   <- RT + A.on_pressed    fired 1 ticks  first @ 15.191 s
```

and `g1_ctrl` then logs, live:

```
FSM: Change state from Passive to FixStand
FSM: Change state from FixStand to Velocity
```

## Consequences recorded elsewhere

* `config/walk_profile.txt` stages every chord: `L2` at t, `L2,up` at t+0.5.
* `ros2/README.md`'s example profile is corrected — the one written in the
  interim block would never have worked.
* An unknown key name already aborts at load. A chord that is *received* but
  arithmetically un-satisfiable did not, and that is what cost the run.
  `fsm_button_probe` exits non-zero when a configured transition never fires,
  so it is usable as a gate rather than only as a debugger.
