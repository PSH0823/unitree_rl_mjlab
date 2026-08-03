# G1 hardware preflight — information to obtain before the first session

**Audience: someone who has never used a Unitree G1.** This document does not
tell you how to run anything (that is
[`g1_first_perception_experiment.md`](g1_first_perception_experiment.md)). It
tells you **what you must know, and write down, before the robot is powered**,
and how to find each answer.

**Why it is a separate document.** Every number in this repository was
measured in simulation or on a bench. Nothing about the real robot, the real
Mid-360, the real network or the real onboard computer has been verified. The
architecture's §12 (Deployment) was written from vendor documentation and is
explicitly marked as unverified; open question **Q-1** ("which exact G1 variant
and onboard PC is in this lab") has been open since Phase 0 and is still open.
Until this checklist is filled in, the hardware path is a *design*, not a
deployment.

**Rule:** fill this in as a copy under
`evidence/hardware/<YYYY-MM-DD>/<session>/preflight.md`. A blank line is an
answer too — "not determined" is information; a guess is not.

**What this phase does NOT do.** It stops at perception-only inference. No
DPCBF output reaches the robot, no velocity command is filtered, no controller
is modified. See §5 for why, and §9 of the first-experiment runbook for what
the later phase needs.

---

## 1. G1 platform

| # | Item | Value | How to get it |
|---|---|---|---|
| 1.1 | Exact G1 variant (EDU / EDU-Ultimate / other; 23-dof or 29-dof) | | Sticker on the robot, purchase record, or Unitree app |
| 1.2 | Controller firmware version | | Unitree app → device info; or the vendor's `firmware_version` service |
| 1.3 | Controller software / SDK version on the robot | | On the onboard PC: `ls /unitree/` , `cat /unitree/*/version*` (paths are vendor-dependent — record what you actually find) |
| 1.4 | Onboard computer model | | `cat /proc/device-tree/model` (Jetson) or `sudo dmidecode -s system-product-name` (x86) |
| 1.5 | CPU architecture | | `uname -m` → `x86_64` or `aarch64` |
| 1.6 | Ubuntu version | | `lsb_release -a`, `cat /etc/os-release` |
| 1.7 | ROS 2 distribution present | | `printenv ROS_DISTRO`, `ls /opt/ros/` |
| 1.8 | RAM | | `free -h` |
| 1.9 | Free disk where bags will be written | | `df -h`, `df -h <bag dir>` |
| 1.10 | sudo/root available? Internet available? | | `sudo -v`; `ping -c1 deb.debian.org` |
| 1.11 | GPU present and usable? | | `nvidia-smi` / `tegrastats` |
| 1.12 | Physical Ethernet interfaces (count, names, which socket is which) | | `ip -br link`, then unplug one cable at a time and re-run |
| 1.13 | Wi-Fi interfaces | | `ip -br link`, `iw dev` |
| 1.14 | Which interface carries the Unitree control network | | Vendor documentation + `ip -br addr`; confirm by watching SDK traffic |
| 1.15 | Is the onboard PC the intended perception computer? | | **Decision, not discovery** — see §3.4 |
| 1.16 | If not: which external workstation, and how is it attached? | | |
| 1.17 | Does the robot already publish body odometry or body velocity? | | `ros2 topic list \| grep -i -E 'odom\|state'` with the SDK domain set; Q-5 |
| 1.18 | Physical E-stop available? Where is it? | | Look at the robot; ask the lab owner. **See §4** |
| 1.19 | Will the robot be tethered / suspended / on a gantry? | | Lab arrangement |

Commands, all read-only, to run and paste verbatim into the record:

```bash
uname -a
uname -m
lsb_release -a
cat /etc/os-release
lscpu
free -h
df -h
ip -br link
ip -br addr
ip route
printenv ROS_DISTRO RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI
```

**Item 1.5 is load-bearing.** The workspace has never been built successfully
on aarch64: an emulated build produced **10 of 18 packages**, stopped by an
upstream `ament_cmake` bug that is characterised but unsolved
(`tools/diagnose_ament_export_libraries.py`). If the answer is `aarch64`,
budget a build slot **before** robot time and read that file and
`tools/build_target.sh` first. Do not treat the emulated result as evidence
the build works.

---

## 2. Mid-360

| # | Item | Value | How to get it |
|---|---|---|---|
| 2.1 | Exact model (Mid-360 vs Mid-360S) | | Sticker |
| 2.2 | Serial number | | Sticker. The last two digits set the IP by Livox convention |
| 2.3 | Firmware version | | Livox Viewer, or the driver's stdout on connect |
| 2.4 | Current LiDAR IP | | Livox Viewer; or `sudo tcpdump -i <iface> -n udp` while it is powered |
| 2.5 | Intended host IP on the LiDAR subnet | | Your choice; must be **actually assigned** to a local interface |
| 2.6 | Subnet mask | | |
| 2.7 | Physical interface the LiDAR is on | | `ip -br addr` after plugging it in |
| 2.8 | Direct to the onboard PC, or through a switch? | | Look at the cabling |
| 2.9 | Power source and voltage | | |
| 2.10 | PTP or other time sync available on this network? | | See the note below |
| 2.11 | Actual mounting position (x, y, z from the pelvis) | | Tape measure — §4 of the experiment runbook |
| 2.12 | Actual mounting orientation (roll, pitch, yaw) | | Tape measure + the floor-plane check |
| 2.13 | Is the sensor mechanically upside down? | | The repository assumes **yes**, roll = π (H-1/H-2). Verify |
| 2.14 | Does the LiDAR→IMU lever arm match the manual? | | Not verified on any unit. See below |
| 2.15 | Packet loss observed? | | The driver's stdout; `ip -s link show <iface>` |

### 2.1 The shipped network config is unusable

`g1_perception_bringup/config/MID360_config.json` currently contains
**upstream sample addresses**:

```
host_net_info.*_ip : 192.168.1.5
lidar_configs[0].ip: 192.168.1.12
```

These are **placeholders, not this robot's**. They are flagged in the file's
own `_comment` block, and `scripts/g1_hw_preflight.sh` exits **2** while they
are present. Leaving them in place produces the worst available failure mode:
the driver starts, logs `bind failed` / `Init lds lidar fail!`, **does not
exit**, creates **no `/livox/*` topic at all** (its publishers are lazy), and
**ignores SIGINT and SIGTERM** — so it looks like a dead sensor rather than a
misconfigured host. Kill it with `pkill -9 -f livo[x]`.

Replace both, then re-run the preflight until it exits 0.

### 2.2 Time synchronisation is a measurement, not a setting

There is **no timestamp-mode field** in `MID360_config.json`. The driver reads
the sync mode out of each packet header; with no PTP/GPS source it falls back
to **the host clock at packet reception**. Which mode is live is only
observable at runtime. `scripts/hw_source_probe.py` reports it (`clock_domain`)
and that report is the answer to §14.3 of the architecture — record it.

### 2.3 The lever arm is an assumed constant

`dlio.yaml` and `t7_hw_extrinsic_guard.py` use the Livox manual's LiDAR→IMU
offset `(0.011, 0.02329, -0.04412) m`, identity rotation. **This has not been
verified against any physical unit.** It is a 5 cm lever arm — irrelevant
while stationary, small under sway. Record that it remains assumed.

---

## 3. Network topology

### 3.1 The three networks

```
              ┌──────────────────────────────────────────────┐
              │             G1 onboard computer              │
              │  (arch/OS/ROS from §1 — NOT yet determined)  │
              │                                              │
   Unitree    │  ┌────────────┐   ┌──────────┐  ┌─────────┐  │
   control  ──┼──┤ NIC A      │   │ NIC B    │  │ Wi-Fi   │  │
   network    │  │ SDK2 DDS   │   │ Livox    │  │ (dev    │  │
   (rt/…)     │  │ rt/lowstate│   │ UDP      │  │  access)│  │
              │  └─────┬──────┘   └────┬─────┘  └────┬────┘  │
              │        │               │             │       │
              │   ┌────┴───────────────┴─────────────┴────┐  │
              │   │  CycloneDDS  (ROS 2 + SDK2, one lib)  │  │
              │   │  pinned by CYCLONEDDS_URI to ONE NIC  │  │
              │   └───────────────────────────────────────┘  │
              └──────────────────┬───────────────────────────┘
                                 │  (optional)
                        ┌────────┴─────────┐
                        │ dev workstation  │  RViz / recording /
                        │                  │  possibly perception
                        └──────────────────┘
```

### 3.2 Interface assignment table — **fill this in**

| Traffic | Protocol | Interface | Address | Notes |
|---|---|---|---|---|
| Unitree SDK2 / controller | CycloneDDS, `rt/…` topics | | | domain must match `ROS_DOMAIN_ID` |
| ROS 2 perception topics | CycloneDDS | | | pinned by `CYCLONEDDS_URI` |
| Livox Mid-360 | raw UDP, ports 56100–56501 | | | not DDS; unaffected by domain |
| Dev workstation link | | | | |

**Do not assume `lo`, `eth0`, or any interface name in this repository.** No
interface name here has been observed on a G1. The `192.168.0.x`/`wlo1`
addresses that appear in worked examples are the *dev machine's*.

### 3.3 The one-NIC question

Determine explicitly whether **one** interface is expected to carry SDK2 DDS,
ROS 2 DDS and Livox UDP simultaneously.

* Livox is plain unicast UDP to the `host_net_info` ports and coexists with
  anything, but it needs the host to actually own the address it names.
* SDK2 and ROS 2 share one CycloneDDS instance by construction (this
  workspace builds one, mitigation R-3), so they can share a NIC — that is the
  documented `unitree_ros2` pattern (§12.2).
* If several interfaces exist, **assign them explicitly** and pin
  `CYCLONEDDS_URI` to the one carrying ROS/SDK traffic. The classic failure
  (recorded in Phase 2) is Cyclone binding loopback: every topic name appears
  in `ros2 topic list`, no data ever crosses to another machine. The preflight
  script hard-fails on a loopback pin for exactly this reason.

### 3.4 Where does perception run? — decide before the session

| | Perception on the onboard PC | Perception on a workstation |
|---|---|---|
| CPU budget (§17.4, <1 core) | measurable, and it is the real number | **not measurable** — record "Orin benchmark" as a named follow-up, do not report a workstation number against the onboard budget |
| Latency (§17.2, p95 ≤ 60 ms) | the real number | inflated by the network |
| Network | LiDAR + DDS local | cloud crosses the wire at ~3 MB/s |
| Build risk | the aarch64 build must work | dev-machine build already works |

Write the decision down. Every later measurement is meaningless without it.

---

## 4. Safety prerequisites — documentation only, and non-negotiable

This phase is **perception-only**: nothing in
`g1_perception_hardware_only.launch.py` publishes a command, and
`test_hw_offline_gates.py` asserts that on every build. The robot is
nevertheless a large machine that can fall over.

**No walking, no command integration, and no stage beyond 12 unless ALL of the
following are true and written down:**

- [ ] a **physical E-stop** exists, and you have located and tested it
- [ ] **a person is assigned to the E-stop** and is doing nothing else
- [ ] the robot is **supported or tethered** for any first motion test
- [ ] the test area is clear — no people in the fall radius, no hard edges
- [ ] obstacles used as props are **soft** (foam/cardboard cylinders)
- [ ] the **controller recovery procedure** is known and has been read aloud
- [ ] the **procedure to return to Passive** (damping) is known
- [ ] the **safe power-down procedure** is known
- [ ] ROS and perception processes can be killed **independently** of the
      controller (they are separate processes; `pkill -9 -f 'component_containe[r]'`
      — the bracketed class is required, an unbracketed pattern matches the
      killing shell's own command line)
- [ ] the Unitree controller's command stream can be stopped **independently**
      of perception

Two facts specific to this stack:

* **Bring-up order for any standing state** (from the walking work): hold the
  robot, `L2+up` → FixStand, `R2+A` → RLBase, *then* lower it. Do **not**
  lower onto FixStand's PD hold pose.
* **The Livox driver ignores SIGINT and SIGTERM** when it fails to bind.
  `ros2 launch` escalates to SIGKILL after 15 s; do not wait for it.

---

## 5. Why this phase stops short of DPCBF walking

Not caution for its own sake — four specific things do not exist yet:

1. **No hardware `RobotState` source.** DPCBF's `Filter()` needs
   `{x, y, φ, v_sagittal, v_lateral}`. In simulation this came from MuJoCo
   ground truth. On hardware it must come from DLIO's `/odom` pose and twist,
   or Unitree's own estimator, or a fusion of both — none of which has been
   built, and DLIO's odometry quality on a walking G1 has never been measured.
2. **No hardware command seam.** The DPCBF insertion point in `deploy` is
   designed (§12.4) but not implemented; `deploy` does not link rclcpp.
3. **No validated common time domain.** The staleness ladder compares
   `frame.stamp` against `t_query`. Whether the LiDAR, DLIO, ROS and the
   controller share a clock is unknown until §2.2 above is measured.
4. **No ground truth.** Every accuracy number in this repository was scored
   against `/sim/gt_obstacles`. On hardware there is none, so the detector and
   the safety inflation are **uncalibrated for hardware** by definition until
   surveyed fixtures are captured.

Feeding an uncalibrated safety filter, driven by unvalidated odometry, across
an unverified time domain, into a real robot's velocity command is the failure
mode this ordering exists to prevent.

---

## 6. Parameters that are NOT calibrated for hardware

Everything below has a value in a YAML file. None of those values is hardware
evidence; several are known to be wrong. **Do not tune any of them from
simulation data and present the result as hardware-valid.**

| Parameter | File | Status |
|---|---|---|
| `measurement_variance: 1.0` | `obstacle_detector.yaml` | **Known wrong.** Asserts a 1-metre 1σ LiDAR measurement; every track is born at σ = 1.0 m. Derive from hardware before touching `k_sigma` |
| `process_variance`, `process_rate_variance` | `obstacle_detector.yaml` | inherited, never fitted |
| `fixed_inflation: 0.051` | `safety_obstacle_filter.yaml` | calibrated **in simulation** on the sim circle-fit bias |
| `k_sigma: 2.748`, `sigma_max` | `safety_obstacle_filter.yaml` | placeholder from an abandoned branch; `use_covariance: false` |
| CropBox bounds (xy ±0.40, z −0.55…0.45) | `cropbox_self_filter.yaml` | **sim-interim**, fitted to wrist returns in a *simulated* grounded pose |
| `min_height: 0.15`, `max_height: 1.60`, `range_min: 0.3` | `pointcloud_to_laserscan.yaml` | Appendix-A values, never checked against a real floor |
| `odom/preprocessing/cropBoxFilter/size: 1.0` | `dlio.yaml` | upstream default; a ±1 m cube that also removes the ground within 1 m |
| `odom/geo/K*` observer gains | `dlio.yaml` | upstream defaults |
| `tracking_duration`, `min_correspondence_cost` | `obstacle_detector.yaml` | Appendix-A, sim-validated only |

Simulation-derived values may be used as **initial values**. Every change must
be recorded with the measurement behind it.

### 6.1 Four different problems that must not be conflated

1. **Self-filtering** — CropBox, in `mid360_link`, removes robot-body returns.
2. **Ground rejection** — `min_height` in `base_footprint` only. There is **no
   ground segmentation**: Patchwork++ is not imported, not built, not
   launched; `/points_no_ground` does not exist; `ground_seg:=patchwork` was a
   silent no-op and is now an explicit error. This works on a **flat floor**
   and is not claimed to work on rough terrain.
3. **Detection/tracking** — the extractor fits circles, the tracker smooths
   them.
4. **Safety inflation** — the safety filter grows those circles.

Making the CropBox bigger to hide a floor artefact, or the inflation bigger to
hide a detector error, mixes two of these and destroys the evidence for both.

### 6.2 Installed-artefact warning

Launch files, YAMLs and the RViz layout are **installed artefacts**. `ros2
launch` reads `install/share/g1_perception_bringup/…`, not the file you edited.
After any edit:

```bash
cd ~/unitree_rl_mjlab_/ros2
colcon build --merge-install --packages-select g1_perception_bringup
source install/setup.bash
```

To prove which copy is live:

```bash
ros2 run g1_perception_bringup config_diff.py
```

It prints source vs installed checksums per file and flags **stale installed
artefacts** — files that survive a rebuild because they no longer exist in
source. `g1_hw_preflight.sh` runs it as a hard gate.

---

## 7. Run the preflight

Once §1–§3 are filled in and `MID360_config.json` carries this robot's
addresses:

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_rl_mjlab_/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<the robot's>
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml    # pinned to the ROS NIC

ros2 run g1_perception_bringup g1_hw_preflight.sh
```

Exit **0** = proceed to stage 1. Exit **1** = a hard failure; each FAIL line
names the experiment stage it blocks. Exit **2** = placeholder network
configuration; go back to §2.1.

The script starts no node, opens no LiDAR socket and publishes no topic. It
cannot move the robot.

**A passing preflight is not a working system.** It clears only what a
stationary machine can check. Whether the LiDAR produces data, whether the
extrinsic matches the physical mount, and whether odometry is sane are
stages 3, 4 and 5 of the experiment runbook.
