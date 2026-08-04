# 첫 번째 G1 perception 실험 — 정확한 runbook

**대상: G1을 처음 다루는 operator.** 아래 각 Stage에는 목적, 사전 조건,
정확한 명령어, 예상 출력, 성공 기준, 실패 징후, 중단 조건, 저장할 파일이
정리되어 있습니다.

**이 session은 perception 전용입니다.** 어떤 perception 출력도 robot의
velocity command로 전달되지 않습니다. 여기서는 controller를 시작하는 항목이
없습니다. `g1_perception_hardware_only.launch.py`의 stack은 command topic을
publish하지 않으며 그렇게 할 수도 없습니다. `hw_offline_gates`는 매 build마다
이를 확인합니다. 이후 DPCBF phase에 필요한 사항과 그것이 이번 session이 아닌
이유는 §9를 참조하십시오.

**먼저 읽을 문서:**
[`g1_hardware_preflight.md`](g1_hardware_preflight.md) — robot의 전원을 켜기 전에
반드시 알아야 할 정보,
[`g1_hardware_code_audit.md`](g1_hardware_code_audit.md) — 검증된 항목과 검증되지
않은 항목.

**Session 규칙(5B checklist에서 이어진 올바른 원칙): debug하지 말고
capture하십시오.** 모든 수치는 bag에서 offline으로 다시 산출할 수 있습니다.
어떤 Stage에서 bag이 생성되지 않았다면 그 Stage는 실패한 것입니다. 이를
기록하고 다음으로 넘어가십시오. robot 사용 시간에는 troubleshooting하지
마십시오.

---

## Environment block — 모든 shell의 맨 위에 붙여넣기

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_rl_mjlab_/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<robot의 값; 기록할 것>
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml     # ROS NIC에 고정
export SESSION=~/unitree_rl_mjlab_/ros2/evidence/hardware/$(date +%F)/s1
mkdir -p "$SESSION"
```

**hardware에서는 모든 곳에서 `use_sim_time`이 false입니다.** 또한 hardware
launch의 argument도 아닙니다. `/clock`이 없으므로 `true`로 남은 node는 시간이
진행하지 않고 timer도 작동하지 않습니다.

**모든 config는 설치된 artefact입니다.** YAML, launch file 또는 RViz layout을
수정한 뒤에는 다음을 실행하십시오.

```bash
cd ~/unitree_rl_mjlab_/ros2
colcon build --merge-install --packages-select g1_perception_bringup
source install/setup.bash
ros2 run g1_perception_bringup config_diff.py     # 반드시 PASS를 출력해야 함
```

---

## Stage 0 — robot 숙지, software 사용 안 함

**목적.** 무엇이든 실행하기 전에 옆에 있는 robot이 어떤 장비인지 숙지합니다.

**사전 조건.** Robot이 현장에 있고, robot 운용 경험자 또는 manual을 이용할 수
있어야 합니다.

**수행 항목(명령어 없음):**

1. **power controls**의 위치와 조작법을 확인하고 순서를 기록합니다.
2. **E-stop**의 위치를 확인합니다. robot 전원을 켜되 stand에 올려둔 상태에서
   한 번 누르고 어떤 일이 일어나는지 확인합니다. session 동안 E-stop을 담당할
   사람을 지정합니다.
3. Robot의 모든 **network port**와 각 port에 연결된 장치를 확인합니다.
4. **onboard PC**의 위치와 shell 접속 방법을 확인합니다.
5. **Mid-360**의 power lead, Ethernet lead, mount를 확인합니다.
6. Robot이 **mechanically supported**(stand, gantry, tether)되어 있는지, 그리고
   session 내내 그 상태를 유지할 수 있는지 확인합니다.
7. **controller의 현재 operating state**(Passive / damping / FixStand / off)를
   확인하고 Passive로 되돌리는 방법을 숙지합니다.
8. **safe shutdown** 절차를 소리 내어 예행 연습합니다.
9. 현장에 있는 모든 사람에게 다음을 소리 내어 말하고 확인받습니다.
   **이 session에서는 robot에 어떤 command도 보내지 않습니다.**

**성공 기준.** 위 모든 항목의 답이 `$SESSION/stage0.md`에 기록되어 있습니다.

**실패 징후.** "필요해지면 E-stop을 찾으면 됩니다."

**중단 조건.** E-stop이 없거나 담당 E-stop operator가 지정되지 않음 → session을
시작하지 않습니다.

**저장할 파일.** `$SESSION/stage0.md`, mount와 cabling 사진.

---

## Stage 1 — target PC 및 network 점검

**목적.** Perception을 실행할 machine과 network가 의도한 대상이 맞는지
입증합니다.

**사전 조건.** Stage 0이 완료되었고 target에서 shell을 사용할 수 있으며
workspace가 build되어 있어야 합니다. Target이 aarch64라면 이는 단순한 형식적
확인이 **아닙니다**(audit §2.2 참조).

**명령어:**

```bash
uname -m
lsb_release -a
ip -br addr
ip route
printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI
df -h "$SESSION"

ros2 run g1_perception_bringup g1_hw_preflight.sh 2>&1 | tee "$SESSION/preflight.txt"
```

**예상 출력.** Section 0–8에 `ok` line이 표시되고 마지막에 `PREFLIGHT PASSED`가
출력됩니다.

**성공 기준.** Preflight exit code가 **0**이고, 특히 다음을 만족해야 합니다.

- loopback이 아닌 올바른 ROS network interface
- `dev lo`가 아닌 route를 사용하며 자체 subnet에 연결된 LiDAR
- **placeholder IP가 없음**
- Livox subnet과 control network 사이에 IP 충돌이 없음
- bag 저장 위치의 여유 공간이 20 GB 이상
- `config_diff.py` PASS(installed == source)

**실패 징후와 의미.**

| Exit / line | 의미 |
|---|---|
| exit 2, `PLACEHOLDER NETWORK CONFIGURATION` | `MID360_config.json`에 upstream sample IP가 남아 있습니다. 이 robot의 값으로 입력하고 rebuild한 뒤 다시 실행하십시오 |
| `RMW_IMPLEMENTATION=<unset>` | 이 shell에서 environment block을 source하지 않았습니다 |
| `CycloneDDS is pinned to loopback` | topic은 보이지만 machine 외부의 data는 전달되지 않습니다 |
| `installed configuration differs from source` | **이전** 수치로 실행하게 됩니다. Rebuild하십시오 |
| `<pkg>/<exe> not installed` | build가 불완전합니다. `tools/build_target.sh`로 돌아가십시오 |

**중단 조건.** 하나라도 hard FAIL이 발생한 경우입니다. Network preflight가
실패한 상태로 진행하지 마십시오. 이후 발생하는 모든 증상의 원인을 sensor로
잘못 판단하게 됩니다.

**저장할 파일.** `$SESSION/preflight.txt`, 위 명령어의 raw output.

---

## Stage 2 — Unitree SDK2와 ROS 2 공존 확인

**목적.** 선택한 interface에서 두 DDS 환경이 모두 작동하고 서로를 방해하지
않는지 확인합니다. **Robot command는 publish하지 않습니다.**

**사전 조건.** Stage 1을 통과했고 robot의 전원이 켜져 있으며 controller가 정상
idle state여야 합니다.

**명령어:**

```bash
# 1. ROS 2 discovery의 기본 작동 확인
ros2 topic list                     | tee "$SESSION/stage2_topics_before.txt"
ros2 doctor --report 2>/dev/null | head -40 | tee "$SESSION/stage2_doctor.txt"

# 2. Unitree state 수신 확인(READ ONLY — publisher 없음)
ros2 topic list | grep -E '^/(rt|lowstate|sportmodestate)' \
                                    | tee "$SESSION/stage2_unitree_topics.txt"
ros2 topic hz /lowstate             # 또는 step 2에서 실제로 확인한 이름

# 3. 공존 smoke test(한 process에서 SDK2와 rclcpp를 link)
ros2 run t10_dds_coexistence t10_smoke | tee "$SESSION/stage2_t10.txt"

# 4. 실제로 load된 CycloneDDS 확인
ldd "$(ros2 pkg prefix rmw_cyclonedds_cpp)/lib/librmw_cyclonedds_cpp.so" \
    | grep -i ddsc      | tee "$SESSION/stage2_ddslibs.txt"
```

**예상 출력.** ROS topic 목록이 표시되고 하나 이상의 Unitree state topic이
존재하며 갱신됩니다. `t10_smoke`가 완료되고 link line에는 정확히 **하나의**
`libddsc.so.0`만 나타납니다(mitigation R-3 — 전체 workspace가 하나의
CycloneDDS를 기준으로 build됨).

**성공 기준.** 선택한 interface에서 두 환경이 동시에 보이고 `t10_smoke`가
통과합니다.

**실패 징후.**

- Unitree topic 없음 → `ROS_DOMAIN_ID`가 잘못되었거나 SDK가
  `CYCLONEDDS_URI`에 고정된 것과 다른 interface를 사용합니다.
- Unitree message type에서 `ros2 topic hz` error 발생 → CLI가 `unitree_hg`
  type을 resolve할 수 없습니다. Phase 4에 기록된 known issue이며 공존 실패가
  **아닙니다**. 기록한 뒤 진행하십시오.
- 두 가지 `libddsc` version이 확인됨 → R-3 위반입니다. 중단하고 rebuild하십시오.

**중단 조건.** ROS 2 discovery가 전혀 작동하지 않는 경우입니다. **이 Stage의
어느 시점에서도 robot command를 publish하지 마십시오.**

**저장할 파일.** 네 개의 `tee` output 전체.

---

## Stage 3 — Mid-360 driver만 실행

**목적.** 처음으로 실제 sensor data를 확인합니다. 다른 항목은 실행하지 않습니다.

**사전 조건.** Stage 1–2를 통과했고 LiDAR network를 사용하는 다른 항목이 없어야
합니다. Perception stack은 실행하지 **않습니다**.

**명령어:**

```bash
# shell A — driver만 실행, DLIO 및 perception 없음
ros2 launch g1_perception_bringup source_hw.launch.py \
    driver:=on lio:=off 2>&1 | tee "$SESSION/stage3_driver.log"

# shell B — 먼저 RECORD하고 나중에 확인
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 3 60 \
    /livox/lidar /livox/imu

# shell B — 그다음 probe 실행
ros2 run g1_perception_bringup hw_source_probe.py --ros-args \
    -p duration:=60.0 -p json:="$SESSION/stage3_source_probe.json" \
    2>&1 | tee "$SESSION/stage3_source_probe.txt"
```

**예상 출력**(probe의 단일 report):

```
  topic                        /livox/lidar
  count                        ~600
  rate_hz                      ~10.0
  stamp_regressions            0
  clock_domain                 host-clock (median offset … ms, stable)
                               ── or ── sensor-clock … (RECORD WHICH)
  ...
  points_per_frame_median      (Mid-360 ≈ 20 000 at 10 Hz — Q-3)
  finite_fraction_min          1.0
PROBLEMS: none detected by this probe
```

**성공 기준(모두 충족):**

- `/livox/lidar`가 10 ± 0.5 Hz이고 `/livox/imu`가 약 200 Hz로 존재함
- 두 topic **모두** `frame_id`가 **`mid360_link`**임(IMU에 `livox_frame`이
  표시되면 patch 0005가 현재 사용하는 build에 반영되지 않은 것임)
- field 7개, `point_step` 26, 보고된 layout deviation 없음
- 두 topic 모두 `stamp_regressions` = 0
- `finite_fraction_min` ≈ 1.0
- 정지 상태에서 IMU median |accel| ≈ 9.81, median |gyro| ≈ 0
- `clock_domain`을 **기록함** — 이것이 §14.3의 답임

위 항목은 예상값일 뿐 그 자체가 acceptance를 의미하지는 않습니다. 정상적인
probe란 probe가 감지할 수 있는 실패가 없다는 뜻입니다.

**실패 징후.**

| 징후 | 의미 |
|---|---|
| 15초 후에도 **`/livox/*` topic이 전혀 없음** | LiDAR 고장이 아니라 SDK bind failure입니다. Driver는 종료되지 않고 SIGINT/SIGTERM을 무시합니다. `pkill -9 -f livo[x]`를 실행하고 IP를 수정한 뒤 preflight를 다시 실행하십시오. 두 번 시도한 후 Stage를 중단합니다 |
| topic은 있으나 0 Hz | 다른 종류의 실패로 device state 또는 firmware 문제입니다. Driver stdout을 저장하고 다음으로 진행하십시오 |
| 잘못된 `frame_id` | `livox_driver.yaml`이 node에 반영되지 않았습니다. Install prefix가 오래된 상태입니다 |
| `stamp_regressions` > 0 | **중단**: tf2와 모든 message filter가 역행하는 stamp를 time jump로 처리합니다 |
| 크거나 계속 변하는 `clock_domain` offset | ROS는 host clock을 사용하지만 LiDAR는 자체 clock 또는 PTP clock을 사용하고 있습니다. 기록하십시오. §9.3에서 해결해야 할 사항이 달라집니다 |
| `arrival_gaps` | packet loss입니다. `ip -s link show <iface>`를 확인하십시오 |
| points/frame이 예상보다 현저히 적음 | Q-3입니다. 기록만 하고 tune하지 마십시오 |

**중단 조건.** 잘못된 frame ID, 역행하는 stamp, 과도한 message age, 불안정한
rate, 예상 밖의 point layout, 타당하지 않은 IMU 값 또는 상당한 packet loss가
발생한 경우입니다.

**저장할 파일.** `stage3_driver.log`, raw bag(**확인하기 전에 먼저
record하십시오. 이 bag은 여러 offline analysis의 입력입니다**), `stage3_source_probe.{txt,json}`,
`env_stage3.txt`, `baginfo_stage3.txt`, `md5_stage3.txt`.

---

## Stage 4 — static TF 및 물리적 extrinsic 검증

**목적.** Publish된 sensor pose가 실제 mount와 일치하는지 확인합니다. 이후의
모든 항목은 이 Stage에서 정의하는 frame을 기준으로 측정됩니다.

**사전 조건.** Stage 3을 통과했습니다. Robot이 평평한 바닥에 정지 상태로 서
있고, 동일한 pose를 재현할 수 있도록 발 위치가 tape로 표시되어 있어야 합니다.

**명령어:**

```bash
# shell A — robot description만 실행
ros2 launch g1_perception_bringup description.launch.py use_sim_time:=false

# shell B
ros2 run tf2_ros tf2_echo base_link torso_link    | tee "$SESSION/stage4_tf_torso.txt"
ros2 run tf2_ros tf2_echo torso_link mid360_link  | tee "$SESSION/stage4_tf_mid360.txt"
ros2 run tf2_ros tf2_echo base_link mid360_link   | tee "$SESSION/stage4_tf_full.txt"
ros2 run tf2_tools view_frames                    # frames.pdf 생성
```

Pelvis origin에서 sensor까지의 **x, y, z, roll, pitch, yaw**를 줄자로 **직접
측정**하십시오. Sketch와 함께 `$SESSION/stage4_measured.md`에 기록합니다.

**예상 출력.** `base_link → mid360_link` ≈ `(-0.0037, 0.00003, 0.4724)`,
roll = π(mount가 **상하 반전**, H-1/H-2), pitch 0.000892입니다.

**Data를 이용한 교차 검증**(Stage 3 bag을 사용해 offline으로 수행):
`mid360_link`의 ground return에 plane을 fitting합니다. 상하가 반전된 mount에서는
ground가 sensor-frame z의 **양수** 방향에 있고 plane normal은 약 **+z**입니다.
반대 결과가 나오면 mount가 H-1이 아니며 이후의 모든 수치를 신뢰할 수 없습니다.

**성공 기준.** 줄자 측정값이 측정 불확도 범위 내에서 publish된 TF와 일치하고
(불확도를 명시하십시오. Robot을 줄자로 측정할 때 최선의 경우에도 ±5 mm입니다),
floor plane의 부호도 예상과 일치합니다.

**실패 징후.** 약 0.47 m의 z offset이 반대 방향으로 나타나거나 plane normal이
잘못된 방향을 가리키면 이 unit에는 roll = π 가정이 맞지 않는 것입니다.

**Mount가 다른 경우 — 다음 순서대로 수정하십시오:**

1. single source of truth인 **`g1_description/urdf/g1_mid360.xacro`**를 수정합니다.
2. 이를 바탕으로 `config/dlio.yaml`의 `extrinsics/baselink2{lidar,imu}`를 다시
   생성하거나 갱신합니다(이 값은 DERIVED이므로 각각 직접 수정하지 않습니다).
3. Guard를 실행합니다: `ros2 run g1_description t7_hw_extrinsic_guard.py` → PASS.
4. `colcon build --merge-install --packages-select g1_description g1_perception_bringup`;
5. `source install/setup.bash`를 실행하고 launch를 다시 시작합니다.
6. `config_diff.py` → PASS.

**두 source를 모두 직접 수정하지 마십시오.** DLIO에는 TF listener가 없습니다.
Extrinsics parameter 자체가 `base_link`의 정의이므로, 오래된 copy는 눈에 보이는
충돌을 일으키는 대신 frame을 조용히 재정의합니다. 또한 extrinsic을
`MID360_config.json`의 `extrinsic_parameter`에 넣지 **마십시오**. 그렇게 하면
두 번 적용됩니다.

**중단 조건.** Extrinsic이 명백하게 잘못된 경우입니다. 계속 진행하지 마십시오.

**저장할 파일.** 세 개의 `tf2_echo` dump, `frames.pdf`,
`stage4_measured.md`, plane-fit 결과, sketch/사진.

---

## Stage 5 — 정지 상태의 DLIO initialisation

**목적.** 다른 항목이 odometry에 의존하기 전에 odometry가 존재하고 안정적인지
확인합니다.

**사전 조건.** Stage 4를 통과했습니다. **처음 3초를 포함해 Stage 전체에서
robot이 완전히 정지해 있어야 합니다.** DLIO는 `odom/imu/calibration/time: 3.0`
동안 IMU bias를 calibration하므로 robot이 움직이면 결과가 오염됩니다.

**명령어:**

```bash
# shell A
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio 2>&1 | tee "$SESSION/stage5_dlio.log"

# shell B — 최소 10분
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 5 600 \
    /livox/lidar /livox/imu /odom /tf /tf_static /diagnostics

# shell C — record 중 실행
ros2 topic hz /odom
ros2 run g1_perception_bringup hw_tf_probe.py --ros-args \
    -p duration:=120.0 -p json:="$SESSION/stage5_tf_probe.json" \
    2>&1 | tee "$SESSION/stage5_tf_probe.txt"
ros2 topic echo /diagnostics --once   | tee "$SESSION/stage5_diagnostics.txt"
top -b -n 3 | head -25                | tee "$SESSION/stage5_cpu.txt"
```

**예상 출력.**

- `/odom` ≈ 100 Hz
- TF `odom→base_link`는 **scan rate인 약 10 Hz** — 이는 fault가 아니라 DLIO의
  설계입니다(broadcast는 100 Hz timer가 아니라 per-scan thread에 있음)
- 처음 약 30개의 cloud가 *"timestamp … earlier than all the data in the
  transform cache"*와 함께 drop됨 — 예상된 startup transient입니다
- diagnostics `perception/dlio` OK, `perception/tf` OK

Bag으로부터 offline으로 다음을 **측정하고 기록**하십시오: `/odom` rate, TF rate,
정지 상태의 x/y/z drift, yaw drift, roll/pitch stability, CPU, memory,
timestamp age, calibration 동작.

**성공 기준.** 기록된 `/odom`에서 ‖p(t) − p(0)‖ / t로 계산했을 때 10분 이상
구간에서 **drift < 1 cm/min**이고, pose jump가 없으며, roll/pitch가 안정적이고,
LiDAR stamp에 TF가 존재합니다(`hw_tf_probe` success fraction ≥ 0.95).

**`/odom`이 존재한다는 이유만으로 유효한 odometry라고 판단하지 마십시오.**
`/odom`은 DLIO가 시작되자마자 존재하며 초기 stamp 값은 실제로 0입니다.

**실패 징후.** Initialisation이 되지 않음, 급격한 pose jump, 상하가 뒤집힌
orientation, 큰 z offset, 증가하는 drift, 일관되지 않은 timestamp, TF 불연속,
비정상적으로 낮은 publish rate.

**중단 조건.** 정지 상태에서 position jump > 1 m → offline 진단을 위해 2분 더
record한 뒤 Stage를 중단합니다.

**저장할 파일.** 10분 이상의 bag, `stage5_dlio.log`,
`stage5_tf_probe.{txt,json}`, `stage5_diagnostics.txt`, `stage5_cpu.txt`.

---

## Stage 6 — 외력으로 움직인 sensor 검증

**목적.** 어떤 항목도 자체 동력으로 움직이기 전에 odometry의 **부호**를
확인합니다. 정지 test로는 반전된 axis를 발견할 수 없습니다.

**사전 조건.** Stage 5를 통과했습니다. Robot을 손으로 안전하게 움직이거나,
들어서 옮기거나, stand 위에서 이동할 수 있어야 합니다. **Robot은 자체 동력으로
움직이지 않습니다.**

**명령어:**

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 6 180 \
    /livox/lidar /livox/imu /odom /tf /tf_static
# 이후 각 동작을 수행할 때마다 log에 알림:
#   1. 전방으로 약 1 m 천천히 평행 이동 후 정지
#   2. 좌측으로 약 1 m 천천히 평행 이동 후 정지
#   3. yaw 방향으로 +90 deg 천천히 회전 후 정지
#   4. 시작 pose로 복귀
ros2 run tf2_ros tf2_echo odom base_link      # 이동 중 실시간 확인
```

**예상 출력.** Robot을 **전방**으로 움직이면 `odom`의 **x**가 증가하고,
**좌측**은 **y**, **위쪽**은 **z**를 증가시킵니다. **위에서 보았을 때 반시계
방향** 회전은 yaw를 증가시킵니다. 이것이 ROS convention입니다(REP-103,
x-forward y-left z-up, right-handed).

**성공 기준.** 네 가지 부호가 모두 올바르고 시작 위치로 돌아왔을 때 pose가
대략 origin으로 복귀합니다(loop error는 측정값이며 gate가 아닙니다).

**실패 징후.** 하나라도 부호가 반전됨 — DLIO bug보다는 Stage 4의 extrinsic
rotation error일 가능성이 가장 높습니다.

**중단 조건.** Axis 또는 부호가 반전된 경우입니다. 진행하지 말고 Stage 4로
돌아가십시오.

**저장할 파일.** Bag, 각 동작의 수행 시각을 적은 log, loop-closure error.

---

## Stage 7 — raw self-hit capture

**목적.** 실제 hardware에서 robot 자체 body가 sensor frame의 어느 위치에
나타나는지 확인합니다. 현재 CropBox는 **simulation에서 유도한 임시값**입니다.

**사전 조건.** Stage 1–3을 통과했습니다. **Perception stack을 실행하지
않습니다.** 이 Stage의 핵심은 filtering되지 않은 cloud입니다. 0.8 m 이내의
모든 물체가 robot 자체뿐이도록 robot 주변 반경 1.5 m 이상을 비우십시오.

**명령어:**

```bash
# driver만 실행 — graph에 CropBox가 전혀 없음
ros2 launch g1_perception_bringup source_hw.launch.py driver:=on lio:=off

# configuration별 bag 하나, 각각 30초 이상, log에 알림
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 7 30 /livox/lidar /tf_static
```

최소한 다음 configuration을 capture하십시오.

1. **안전한 경우에 한해**, 전원은 켰지만 서 있지 않은 상태
2. Passive (damping);
3. FixStand;
4. Arm을 nominal pose에 둔 상태
5. 대표적인 arm position(각 arm을 전체 range에 걸쳐 천천히 sweep)
6. 대표적인 torso pitch(waist yaw/roll/pitch를 range 전반에서 변경)
7. Wrist를 LiDAR 가까이 가져온 상태 — simulation에서 문제로 표시된 case

**Offline analysis(session 도중이 아니라 종료 후 수행):**

```bash
ros2 run g1_perception_bringup selfhit_analysis.py "$SESSION/stage7_<cfg>" \
    --radius 0.8 --json "$SESSION/stage7_selfhit.json"
```

**예상 출력.** Arm과 torso가 있는 위치에 near-range return이 모여 있고, 0.8 m
이내에는 그 외의 물체가 없습니다.

**성공 기준.** Head shell, torso, shoulder, wrist, thigh, knee, foot, cable,
bracket에서 발생한 return을 식별한 report가 작성되어 있습니다. 각 항목에는
`mid360_link`에서의 point 위치, robot으로부터의 거리, frame 간 지속성, 그리고
geometry masking, CropBox, `range_min`, height filter 중 **어떤 mechanism으로
제거할 것인지**가 포함되어야 합니다.

**CropBox를 즉시 키우지 마십시오.** 먼저 정량화하십시오. Analysis가 출력하는
결정적인 교차 검증은 다음과 같습니다. **|x| 또는 |y|의 99.9th percentile이
`range_min`(0.30 m)에 가까우면 arm이 detection band 안으로 들어오는 것이며,
해결책은 더 큰 box가 아니라 shaped mask 또는 pose-aware mask입니다.** Box를
키우면 robot의 근거리 시야를 희생해 self-filtering을 확보하게 됩니다.

**실패 징후.** Near field를 훼손하지 않고는 어떤 box로도 제외할 수 없는
반경에서 return이 발생합니다.

**중단 조건.** 없음 — 이 Stage에서는 record만 수행합니다.

**저장할 파일.** Filename에 configuration 이름을 넣은 configuration별 bag,
`stage7_selfhit.json`, 작성한 report, 각 pose의 사진.

---

## Stage 8 — CropBox만 단독 검증

**목적.** Self-filter가 robot은 제거하면서 **실제 near-field obstacle은
유지하는지** 확인합니다.

**사전 조건.** Stage 7 capture를 완료했고 부드러운 외부 물체를 준비했습니다.

**명령어:**

```bash
# source + TF + perception container(CropBox가 첫 번째 Stage)
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio

# input과 output 비교
ros2 topic hz /livox/lidar /points_self_filtered
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 8 60 \
    /livox/lidar /points_self_filtered /tf /tf_static /diagnostics
```

부드러운 물체를 robot으로부터 **0.4, 0.6, 0.8, 1.0, 1.5 m** 거리에 놓고,
각각 30초씩 측정하면서 log에 알리십시오.

**예상 출력.** `/points_self_filtered`는 동일한 10 Hz에서 더 적은 point를
포함하고 diagnostics `perception/self_hit`은 OK입니다.

**성공 기준 — 두 항목을 모두 충족해야 하며 두 번째 항목을 빠뜨리기 쉽습니다:**

1. Robot-body return이 제거됩니다.
2. `range_min` 이상의 모든 거리에서 외부 물체가 **계속 존재합니다.**

**실패 징후.** CropBox가 모든 near-field data를 제거합니다. 이는 pass가 아니라
깨끗해 보일 뿐 실제로는 근거리를 볼 수 없는 상태입니다.

**중단 조건.** 하나의 box로 (1)과 (2)를 동시에 만족할 수 없다면 중단하고,
shaped/pose-aware mask가 필요하다고 기록하십시오. 이는 tuning 변경이 아니라
design 변경입니다.

**저장할 파일.** Bag, frame별 point-count-in/out, 거리별 object visibility table,
`stage8_diagnostics.txt`.

---

## Stage 9 — height band 및 LaserScan 검증

**목적.** 실제 바닥에서 2-D projection이 무엇을 보는지 확인합니다.

**사전 조건.** Stage 8을 통과했고 바닥이 평평해야 합니다.

**정확한 semantics**(혼동하기 쉬운 항목):

- **CropBox bound는 `mid360_link`를 기준으로 합니다**(cloud frame이며 TF는
  관여하지 않음).
- **`min_height` / `max_height`는 `base_footprint`로 transform한 뒤
  적용됩니다.**
- **`range_min`은 `base_footprint`에서의 수평 거리입니다.**
- **Patchwork++와 ground segmentation은 없습니다.** `min_height: 0.15`가
  유일한 floor rejection입니다. 이는 height band이며 평평한 바닥에서만
  유효합니다.

**명령어:**

```bash
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio use_rviz:=true
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 9 60 \
    /livox/lidar /points_self_filtered /scan /tf /tf_static /diagnostics
```

Scene별로 30초 이상 측정하십시오: 물체가 없는 평평한 바닥, 규격을 아는 낮은
obstacle, 중간 높이의 cylinder, table leg, 필요한 경우 table edge/overhang,
robot 자체의 leg와 foot, 중간 정도의 body pitch, 그리고 **안전한 경우** 작은
바닥 경사.

**측정 항목:** occupied scan-bin fraction, floor-return fraction, self-return
fraction, object 높이에 따른 detection loss, frame 간 flicker.

**성공 기준.** 물체가 없는 평평한 바닥에서 실내 scan은 사실상 비어 있고,
band의 lower edge보다 높은 물체가 올바른 range에 나타나며, near-field ring을
floor return이 지배하지 않습니다(diagnostics `perception/floor_artifact` OK).

**실패 징후.** Near-field return의 ring이 지속적으로 나타남(floor가 band에
들어오거나 body pitch 때문에 band가 기울어 floor와 겹침), body가 pitch할 때
물체가 사라짐.

**중단 조건.** 없음. 단, **rough terrain을 지원되는 feature로 간주하여 test하지
마십시오.** 현재 지원되는 feature가 아니므로 rough-terrain capture를 결과로
제시하면 허위 주장이 됩니다.

**저장할 파일.** Bag, scene별 metric table, RViz screenshot.

---

## Stage 10 — obstacle extractor 검증

**목적.** 측량한 geometry를 기준으로 실제 detector error 수치를 처음
측정합니다.

**사전 조건.** Stage 9를 통과했고 robot이 tape로 표시된 pose에서 정지해
있습니다. **측량한 원형 fixture**로 cylinder 3개 이상을 사용하며, 하나 이상은
r ≥ 0.30 m이고 **모든 prop은 r ≤ 0.52 m**여야 합니다. 이보다 크면 거리에서
circle-fit gate가 drop합니다(측정 결과: r = 0.55 m는 2 m에서 detect되지만
3 m와 4 m에서는 drop됨).

Survey convention: **prop face**까지 측정하고, centre = bearing 방향의 face
distance + r로 정의합니다. `$SESSION/t4_layout.yaml`에 기록하십시오.

```yaml
match_radius: 0.5
targets:
  - {name: cyl_1m,     x:  0.813, y:  0.813, r: 0.15}
  - {name: cyl_2m,     x: -1.520, y:  1.520, r: 0.15}
  - {name: blocker_3m, x:  2.333, y: -2.333, r: 0.30}
```

**각 fixture의 기록 항목:** true centre, true radius, distance, visibility
(full arc / partly occluded), height, material.

**Frame 주의.** Pipeline은 obstacle을 **`odom` frame**으로 publish합니다
(`obstacle_detector.yaml`의 `frame_id: odom`, extractor와 tracker 모두).
따라서 `ros2 topic echo /tracked_obstacles`가 출력하는 x, y는 줄자로 잰
"로봇 앞 2 m"와 직접 비교할 수 없습니다. 아래 두 case 모두 이 변환을
대신 수행합니다 — case A는 tf2 lookup 후 숫자를 출력하고, case B는 RViz의
Fixed Frame을 `base_link`로 두고 그림으로 보여줍니다.

`t4_layout.yaml`의 target은 지금까지와 동일하게 **`odom` frame**으로
기록합니다(offline T4 harness가 그 convention을 사용하며, odom→base_link
변환 오차를 detector 오차에 섞지 않기 위함).

### Case A — 모니터 없음(SSH 한 대): 로봇 좌표계 콘솔 출력

Onboard PC에 화면이 없거나 SSH만 연결된 상태에서 사용합니다. Perception이
추론한 obstacle을 **로봇 기준 좌표계**로 변환해 표로 출력합니다.

Shell 1 — stack:

```bash
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio use_rviz:=false
```

Shell 2 — 추론 결과 read-out(publish하지 않으며 로봇을 움직일 수 없음):

```bash
ros2 run g1_perception_bringup hw_obstacle_watch.py --ros-args \
    -p target_frame:=base_link -p rate:=2.0 \
    -p layout:="$SESSION/t4_layout.yaml" \
    -p json:="$SESSION/stage10_watch.jsonl" -p duration:=60.0
```

출력 형태(`x`, `y`, `range`, `bearing`은 `base_link` 기준, `radius`는
margin이 더해진 값, `true_radius`는 실측 반지름):

```
=== t+  12.5 s   frame=base_link   (source: obstacle_detector publishes in odom)
--- /raw_obstacles        10.0 Hz  age 0.05 s  3 circle(s)
--- /tracked_obstacles    10.0 Hz  age 0.04 s  3 circle(s)
    uid       x       y   range  bearing       r  true_r      vx      vy
      7    1.02   -0.03    1.02      -1.7   0.201   0.150    0.00    0.00
      9    1.98    0.05    1.98       1.4   0.203   0.152    0.00    0.00
     11    2.94   -0.11    2.94      -2.1   0.352   0.301    0.01   -0.01
  -- vs surveyed layout (odom frame) --
  cyl_1m       HIT   centre err  0.031 m   true_r 0.150 (err +0.000)   r 0.201
  cyl_2m       HIT   centre err  0.048 m   true_r 0.152 (err +0.002)   r 0.203
  blocker_3m   HIT   centre err  0.083 m   true_r 0.301 (err +0.001)   r 0.352
--- /obstacles_safe        1.0 Hz  age 0.42 s  3 circle(s)
```

읽는 법:

- **`layout:=`을 주면 HIT/MISS/EXTRA 줄이 붙습니다.** MISS는 missed
  detection, EXTRA는 false positive 후보이며, `centre err`와 `true_r err`는
  Stage 10 성공 기준(≤ 0.10 m, ≤ 0.05 m)과 그대로 비교할 수 있는 값입니다.
  단, **`true_radius` 기준은 `/tracked_obstacles` 줄에서만** 읽으십시오
  (`/obstacles_safe`는 §9.6 inflation이 이미 적용된 값입니다).
- `layout:=`을 생략하면 표만 출력되므로, 측량 전에 "무엇이 보이는지"만
  확인하는 용도로 쓸 수 있습니다.
- **`(TF: latest)`나 `(TF FAILED: ...)`** 가 header에 찍히면 그 frame의
  좌표는 신뢰하지 마십시오. TF tree가 message stamp를 따라오지 못한
  것이므로 Stage 4/5의 `hw_tf_probe.py`로 돌아가야 합니다.
- `vx`/`vy`는 **odom 기준 속도를 로봇 축으로 회전만 시킨 값**입니다(로봇
  자기 속도를 빼지 않음). 정지한 prop은 로봇이 걸어도 0에 가깝습니다.
- `NO DATA`는 node가 죽은 것이고 `0 circle(s)`은 살아 있으나 아무것도
  검출하지 못한 것입니다. 후자라면 `/scan`부터 확인하십시오(Stage 9 영역).

Bag은 평소대로 별도 shell에서 함께 기록합니다:

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 10 60 \
    /livox/lidar /scan /raw_obstacles /tracked_obstacles /obstacles_safe \
    /odom /tf /tf_static /diagnostics
```

### Case B — onboard PC에 모니터가 연결된 경우: RViz

`use_rviz:=true`는 committed layout(`perception.rviz`)을 여는데, 그 layout은
**Fixed Frame이 `odom`**이고 GT/DPCBF overlay 중심입니다. Stage 10에서는
로봇 기준으로 보는 편이 낫기 때문에 **robot-frame layout**을 함께
설치해 두었습니다(`rviz/perception_robot_frame.rviz`: Fixed Frame
`base_link`, 로봇이 원점에 고정, 1 m grid, `/scan` + raw/tracked/safe layer
ON, GT·DPCBF overlay 없음, Measure tool 포함).

```bash
RVIZ_CFG=$(ros2 pkg prefix g1_perception_bringup)/share/g1_perception_bringup/rviz/perception_robot_frame.rviz
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio use_rviz:=true rviz_config:="$RVIZ_CFG"
```

원격 데스크톱이 아니라 실제 모니터에 띄우는 것이므로 로그인 세션의
`DISPLAY`가 필요합니다. SSH에서 실행한다면:

```bash
export DISPLAY=:0      # onboard PC의 실제 X display
xdpyinfo -display :0 >/dev/null || echo "X display가 없음 → case A를 쓰십시오"
```

화면에서 확인할 것:

- **주황 원(TrackedObstacles)의 중심이 실제 prop 위치와 일치**하는지. 원마다
  uid label이 붙습니다.
- **회색 원(RawObstacles)** 은 frame별 extractor 출력이므로 깜빡임(flicker)과
  split/merge를 여기서 봅니다.
- **빨간 원(SafeObstacles)** 은 §9.6 gating + inflation 후의 값이라 항상 더
  큽니다. **이 반지름을 detector 정확도로 읽으면 안 됩니다.**
- **노란 점(`/scan`)** 은 extractor의 입력입니다. Prop이 원으로 잡히지 않을
  때 애초에 scan에 있었는지를 여기서 판단합니다.
- 거리는 1 m grid 또는 Measure tool로 읽고, screenshot을
  `$SESSION/screenshots/`에 저장하십시오.

RViz를 띄우더라도 **case A의 `hw_obstacle_watch.py`는 함께 돌리는 것을
권장합니다.** 화면은 "맞아 보인다"까지만 말해 주고, 보고서에 넣을 수치는
콘솔 표에서 나옵니다.

1, 2, 3 m에서 반복하십시오(또는 세 개를 한 번에 배치하고 bag 하나를
생성하십시오).

**Offline — simulation gate와 동일한 harness를 hardware bag에 지정:**

```bash
T4_BAG="$SESSION/stage10_<t>" T4_LAYOUT="$SESSION/t4_layout.yaml" \
T4_USE_SIM_TIME=false \
  launch_test src/g1_perception/g1_perception_bringup/test/test_detection_static.launch_test.py
```

**측정 항목:** detection probability, centre error, fitted-radius error,
false positive, missed detection, merged circle, split circle, partial
visibility의 영향.

**성공 기준(simulation gate를 hardware 기준으로 다시 기술 — target으로
취급하고 충족 여부와 관계없이 수치를 보고):** centre error ≤ 0.10 m,
`/tracked_obstacles`의 pre-inflation 상태에서 `true_radius` error ≤ 0.05 m,
detection latency ≤ 2 frame.

**다음 사항도 명시적으로 기록하십시오:** Simulation에서 확인한 "radius가
커질수록 bias가 증가한다"는 결과(r = 0.30에서 radius +33 mm, centre 83 mm)가
재현됩니까? `fixed_inflation = 0.051` calibration은 전적으로 이 결과에
근거합니다.

**실패 징후.** 1 m에서 circle이 하나도 detect되지 않음 → 먼저 `/scan`을
확인하십시오(Stage 9 영역).

**중단 조건.** 없음. 실패한 경우 offline extractor 작업을 위해 `/scan` +
`/raw_obstacles`를 60초간 record하고 Stage를 종료합니다.

**Detector error를 숨기기 위해 safety inflation을 수정하지 마십시오.** Detector
calibration과 safety inflation은 각각 별도의 evidence가 필요한 별개의 작업입니다.

**저장할 파일.** Bag, `t4_layout.yaml`, `stage10_watch.jsonl`(case A의 live
read-out), case B를 수행했다면 RViz screenshot, harness output, target별
error table, 측량한 layout 사진.

---

## Stage 11 — tracker 검증

**목적.** Track stability와 velocity accuracy, 그리고 publish된 covariance가
유의미한지 확인합니다.

**사전 조건.** Stage 10을 완료했습니다.

**Scenario**별로 60초 이상 측정하십시오: robot(또는 sensor)이 움직이는 동안의
static obstacle, robot이 정지한 동안의 moving obstacle, 두 obstacle의 교차,
일시적인 occlusion, occlusion 후 재등장.

**명령어:**

```bash
ros2 run g1_perception_bringup hw_record.sh "$SESSION" 11 60 \
    /scan /raw_obstacles /tracked_obstacles /obstacles_safe /odom /tf /tf_static
# replay 중 offline으로 실행:
/usr/bin/python3 src/g1_perception/g1_perception_bringup/test/phase4_obstacles_dump.py \
    120 "$SESSION/stage11.jsonl"
```

**측정 항목:** track-ID stability, velocity RMSE, association failure, ID
switch, track creation delay, track deletion delay, covariance output,
coasting behaviour, merged-track behaviour.

**성공 기준.** ID가 짧은 occlusion 동안 유지되고 velocity error가 정량적으로
규명됩니다(충족해야 할 기존 hardware 수치는 없음).

**`measurement_variance: 1.0`은 calibration되지 않은 값으로 취급해야 합니다.**
이 session의 data에서 실제 값을 산출하십시오. 가급적 각 target의 mean을
중심으로 한 detector 자체의 scatter를 사용합니다(`fixed_inflation`이 이미
처리하는 systematic circle-fit bias를 흡수하므로 중복 계산하면 안 됨).

```bash
/usr/bin/python3 src/g1_perception/g1_perception_bringup/test/measure_measurement_variance.py \
    "$SESSION/stage11.jsonl"
```

**Covariance consistency를 평가하기 전에는 `use_covariance`를 enable하지
마십시오.** `calibrate_k_sigma.py`는 잘못된 `R`이 조용히 흡수되지 않도록
σ p50 > 0.25 m인 동안 `k_sigma`를 출력하지 않습니다. 먼저 `R`을 설정한 뒤
containment 수치를 다시 검증하십시오. `R`은 tracker가 각 measurement를
신뢰하는 정도를 바꾸며, 그에 따라 전체 safety chain이 사용하는 tracked
position도 달라집니다.

**중단 조건.** 없음 — 이 Stage는 측정 과정입니다.

**저장할 파일.** Bag, `stage11.jsonl`, variance output, 출력된 경우 `k_sigma`
output, scenario별 table.

---

## Stage 12 — 전체 perception-only pipeline

**목적.** 이 phase의 최종 단계로, robot에는 아무것도 연결하지 않은 상태에서
실제 data로 전체 chain을 실행합니다.

**사전 조건.** Stage 1–11을 완료했거나 각 실패를 기록했습니다.

**명령어:**

```bash
ros2 launch g1_perception_bringup g1_perception_hardware_only.launch.py \
    driver:=on lio:=dlio use_rviz:=true record:=on \
    bag_path:="$SESSION/stage12_full" 2>&1 | tee "$SESSION/stage12.log"

# 실행 중 다른 shell에서 수행
ros2 topic hz /livox/lidar /points_self_filtered /scan \
              /raw_obstacles /tracked_obstacles /obstacles_safe
ros2 run g1_perception_bringup hw_tf_probe.py --ros-args \
    -p duration:=120.0 -p json:="$SESSION/stage12_tf.json"
/usr/bin/python3 src/g1_perception/g1_perception_bringup/test/phase4_latency_probe.py 60 \
    | tee "$SESSION/stage12_latency.txt"
/usr/bin/python3 src/g1_perception/g1_perception_bringup/test/proc_cpu.py \
    | tee "$SESSION/stage12_cpu.txt"
ros2 topic echo /diagnostics | head -120 | tee "$SESSION/stage12_diagnostics.txt"
```

**다음 여섯 topic이 모두 live인지 확인하십시오:** `/livox/lidar`, `/points_self_filtered`,
`/scan`, `/raw_obstacles`, `/tracked_obstacles`, `/obstacles_safe`.

**Command가 publish되지 않음을 입증하십시오**(가정하지 말고 직접 수행):

```bash
ros2 topic list | grep -E 'cmd|lowcmd|wireless|sport' \
    | tee "$SESSION/stage12_no_command_topics.txt"     # 예상: 이 stack에서 출력 없음
ros2 node list  | tee "$SESSION/stage12_nodes.txt"     # 예상: g1_ctrl, deploy 없음
ros2 topic info /obstacles_safe --verbose \
    | tee "$SESSION/stage12_safe_subscribers.txt"      # 예상: RViz relay만 존재
```

**측정 항목:** end-to-end rate, cloud→scan, cloud→raw, cloud→tracked,
cloud→safe latency, process별 CPU, memory, TF miss rate, message age, packet
loss, obstacle count, false positive, floor artefact, self-hit artefact.

**성공 기준(모두 충족):**

- 실제 sensor input이 안정적임
- odometry가 안정적임
- timestamp가 일관된 TF(cloud stamp에서 `hw_tf_probe` success ≥ 0.95)
- **지속적으로 나타나는 robot-body phantom obstacle이 없음**
- **지속적으로 나타나는 floor obstacle ring이 없음**
- 규격을 아는 fixture가 올바른 `odom` coordinate에서 detect됨
- Sensor 또는 robot이 움직이는 동안 static world obstacle이 `odom`에서 대략
  정지 상태로 유지됨
- **actuation 없음** — 위의 세 검사가 아무것도 반환하지 않음
- 참고용 budget(알 수 없는 hardware에 적용할 gate가 아니라 dev-machine 수치):
  perception container < 1 core, cloud→`/obstacles_safe` p95 ≤ 60 ms.
  Perception을 onboard PC가 아닌 workstation에서 실행했다면 **이를 명시하고**
  on-target benchmark를 후속 항목으로 분명하게 기록함

**실패 징후.** Robot을 따라다니는 phantom obstacle(self-hit가 CropBox를 통과),
고정 반경의 obstacle ring(floor가 height band에 들어옴), sensor가 움직일 때
`odom`에서 drift하는 obstacle(detector error가 아니라 odometry 또는 extrinsic
error).

**중단 조건.** Command topic이 존재한다는 증거가 하나라도 확인됨 → 즉시
중단하고 무엇이 이를 시작했는지 확인합니다.

**저장할 파일.** Bag과 해당 **`.session.json` 및 `configs/`**, 위의 모든 `tee`
output, RViz screenshot, latency 및 CPU dump.

---

## Stage 13 — 제어된 G1 posture 변경, walking 없음

**목적.** Robot이 실제로 body를 움직일 때 perception stack의 동작을 확인합니다.
**Translation command는 보내지 않습니다.**

**사전 조건.** Stage 0–12를 통과했고 E-stop operator가 지정 위치에 있으며,
robot이 지지된 상태이고, 모든 관계자에게 작업 내용을 알렸습니다.

**절차.** **표준 Unitree 절차**를 사용해 안전하다고 알려진 standing state로
진입합니다(robot을 잡은 상태에서 `L2+up` → FixStand, `R2+A` → RLBase,
*그다음* robot을 내림 — FixStand의 PD hold pose 상태로 내리지 마십시오).
이후 perception을 record하면서 다음을 수행합니다.

- body sway
- 작은 pitch
- 작은 yaw
- arm motion
- posture transition

**명령어:** Stage 12와 동일하며, `hw_record.sh`를 사용해 motion type별 bag을
추가로 생성합니다.

**측정 항목:** self-hit의 재등장 여부, floor artefact의 증가 여부, cloud stamp에서
TF가 계속 사용 가능한지, DLIO가 안정적으로 유지되는지, obstacle track이
jump하는지, `base_footprint`가 계속 유효한지.

**성공 기준.** 모든 motion에서 perception과 odometry가 안정적으로 유지되고,
성능 저하가 단순히 관찰되는 데 그치지 않고 정량적으로 규명됩니다.

**실패 징후.** Body motion 중 DLIO가 diverge함, TF rate가 낮아지면서 TF lookup이
실패함, body가 pitch할 때 track이 jump함(고정된 `base_link→torso_link`
approximation은 실제 hardware error source입니다. Waist joint는 움직이지만
publish된 transform은 움직이지 않음).

**중단 조건.** Perception 또는 odometry가 불안정해지면 **즉시 중단**하고
Passive로 돌아갑니다.

**저장할 파일.** Motion type별 bag, diagnostics stream, before/after 비교 결과.

---

## Stage 14 — OPTIONAL 외부 command를 이용한 초저속 motion

**기본적으로 차단되어 있습니다.** 이 Stage는 operator, lab, robot-safety
procedure에서 **명시적으로** 허용한 경우에만 실행합니다.

**목적.** Gait 중 odometry, self-hit, floor artefact, TF timing, obstacle
stability, CPU를 확인합니다. Gate가 아닌 관찰 항목입니다.

**사전 조건.** Stage 13을 통과했고 E-stop operator가 지정 위치에 있으며 주변
공간이 비어 있습니다. 기존 Unitree control interface가 **매우 낮으며 사전에
알고 있는** velocity를 command합니다. **Perception은 read-only로 유지됩니다.**

**반드시 지킬 제약 조건:**

- **`/obstacles_safe`는 controller에 연결되지 않습니다.** Visualisation relay
  외에는 어떤 항목도 subscribe하지 않습니다.
- **DPCBF는 실행되지 않습니다.** 이 Stage를 근거로 DPCBF가 hardware에서
  작동한다고 주장하지 마십시오. Filtered command는 존재하지 않습니다.
- Perception launch는 변경하지 않으며, command는 완전히 별도로 robot 자체
  interface에서 전달됩니다.

**측정 항목:** gait 중 odometry, arm 및 leg motion 중 self-hit 동작, floor
artefact, TF timing, `odom`에서의 obstacle stability, locomotion 중 CPU 및
latency.

**중단 조건.** 불안정성이 하나라도 발생하거나, 예상하지 못한 motion이
발생하거나, control loop에 포함되었을 때 문제가 될 perception degradation이
발생한 경우입니다.

**저장할 파일.** Bag, command한 velocity profile, metadata에
`robot_motion_occurred: true`를 설정한 Stage 12와 동일한 metric set.

---

## Session artefact

각 session마다 다음 항목을 포함하는
`evidence/hardware/<date>/<session_name>/`을 생성하십시오.

```
preflight.txt              stage0.md … stage14.md      command log
env_stage*.txt             (capture별 environment dump)
<bag>/                     <bag>.session.json          configs/
baginfo_stage*.txt         md5_stage*.txt
stage*_source_probe.json   stage*_tf_probe.json
stage*_diagnostics.txt     stage*_cpu.txt   stage*_latency.txt
frames.pdf                 screenshots/*.png
t4_layout.yaml             측정한 fixture geometry
stage10_watch.jsonl        robot-frame live read-out (hw_obstacle_watch.py)
operator_notes.md          known_failures.md
```

여기에 `hw_session_metadata.py`가 작성하고 probe JSON을 바탕으로 offline에서
완성하는 machine-readable summary를 추가합니다.

```json
{
  "hardware": {
    "g1_variant": "",
    "target_arch": "",
    "lidar_serial": "",
    "lidar_firmware": ""
  },
  "network": {
    "ros_interface": "",
    "livox_interface": "",
    "ros_domain_id": 0
  },
  "results": {
    "lidar_rate_hz": null,
    "imu_rate_hz": null,
    "odom_rate_hz": null,
    "tf_lookup_success": null,
    "cloud_to_safe_p95_ms": null
  },
  "actuation_enabled": false
}
```

Record에는 **robot motion 발생 여부**와 **활성 상태였던 command publisher의
존재 여부**도 명시해야 합니다(`session.robot_motion_occurred`,
`session.command_publisher_active` — 둘 다 기본값은 false이며 recording tooling이
metadata의 일부로 보고함).

Session 종료 시 다음을 확인하십시오.

- [ ] 모든 bag에 대해 `md5sum`과 size를 확인하고 생성 recipe 및 목적과 함께
      목록으로 정리함
- [ ] Date, operator, preflight 답변, topology 결정, Stage별 결과를 **수치와
      함께**, 중단한 항목과 그 이유를 session log에 작성함
- [ ] Prop과 survey measurement를 사진 또는 sketch로 기록함
- [ ] **Robot에서 어떤 항목도 tune하지 않음** — 모든 retune은 측정값을
      기록하면서 bag을 이용해 offline으로 수행함

---

## Shutdown

```bash
# 1. recording 중단(recording shell에서 Ctrl-C), rosbag이 마무리될 때까지 대기
# 2. perception launch 중단(shell A에서 Ctrl-C)
# 3. 일부 path에서 driver가 SIGINT/SIGTERM을 무시해 종료되지 않을 수 있음
pkill -9 -f 'livo[x]'
pkill -9 -f 'component_containe[r]'     # bracket는 필수: bracket가 없는 pattern은
                                        # kill을 실행하는 shell 자체의 command line과
                                        # match됨
pkill -9 -f 'dlio_odom_nod[e]'
ros2 node list                          # 예상: 비어 있음
# 4. robot 자체 절차에 따라 controller를 Passive로 복귀
# 5. Stage 0 절차에 따라 전원 끄기
```

---

## 처음 사용하는 operator에게 필요한 빠른 답변

| 질문 | 답변 |
|---|---|
| 어떤 computer에서 perception을 실행합니까? | Preflight §3.4에 기록한 결정에 따릅니다. 이 repository에서 결정하지 않습니다 |
| 어떤 network interface를 사용합니까? | Preflight §3.2 table을 따르며 `CYCLONEDDS_URI`를 통해 고정합니다 |
| G1과 Mid-360의 IP address는 무엇입니까? | Preflight §1/§2를 참조하십시오. 제공된 값은 **placeholder**입니다 |
| CycloneDDS는 어떻게 구성합니까? | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, SDK2와 일치하는 `ROS_DOMAIN_ID`, 하나의 NIC를 고정하는 `CYCLONEDDS_URI`를 사용합니다 |
| Target에서 workspace를 어떻게 build합니까? | `tools/build_target.sh`를 사용합니다(먼저 `--check`). aarch64는 **검증되지 않았습니다** |
| Mid-360만 어떻게 launch합니까? | `source_hw.launch.py driver:=on lio:=off`(Stage 3) |
| LiDAR와 IMU를 어떻게 검증합니까? | `hw_source_probe.py`(Stage 3) |
| Mount extrinsic을 어떻게 확인합니까? | `tf2_echo` + 줄자 측정 + floor-plane 부호(Stage 4) |
| DLIO를 어떻게 initialise합니까? | Robot을 3초 이상 정지하고 `lio:=dlio`를 사용합니다(Stage 5) |
| LiDAR timestamp에서 TF를 어떻게 확인합니까? | `hw_tf_probe.py` — latest가 아닌 stamped lookup을 사용합니다 |
| CropBox 동작을 어떻게 점검합니까? | Stage 8에서 외부 물체를 두고 `/livox/lidar`와 `/points_self_filtered`를 비교합니다 |
| Floor return을 어떻게 식별합니까? | Stage 9의 near-field `/scan` bin fraction과 diagnostics `perception/floor_artifact`를 사용합니다 |
| Actuation 없이 전체 stack을 어떻게 launch합니까? | `g1_perception_hardware_only.launch.py`를 사용합니다 |
| Metadata와 함께 bag을 어떻게 record합니까? | `record:=on` 또는 `hw_record.sh`를 사용합니다 |
| Calibration되지 않은 parameter는 무엇입니까? | Preflight §6을 참조하십시오 |
| 어떤 조건에서 실험을 중단합니까? | 각 Stage의 중단 조건과 phase report §11.5를 따릅니다 |
| 전체 system을 어떻게 shutdown합니까? | 위의 Shutdown block을 따릅니다 |
| 아직 DPCBF walking을 하지 않는 이유는 무엇입니까? | Preflight §5와 아래 §9를 참조하십시오 |

---

## §9 — 이후 DPCBF hardware phase에 필요한 사항

**Design note일 뿐입니다. 아래 내용은 구현되지 않았으며 이번 session의 scope에
포함되지 않습니다.**

### 9.1 Hardware `RobotState` source

DPCBF의 `Filter()`에는 다음이 필요합니다.

```
RobotState { x; y; phi; sagittal_velocity; lateral_velocity; }
```

Candidate source: DLIO `/odom` pose, DLIO `/odom.twist`, 미분한 pose,
Unitree의 body-velocity estimator, body IMU, fused estimator.

Design 단계에서 측정을 통해 다음을 해결해야 합니다: 각 항목의 **frame**,
각 항목이 가지는 **timestamp**와 clock 공유 여부, **latency**, **velocity
noise**(10 Hz-TF pose를 미분하는 것은 twist를 읽는 것과 같지 않음), DLIO와
robot 자체 estimate 간 **yaw consistency**, **odometry loss 시 동작**. 마지막
항목은 정의되지 않은 동작이 아니라 명확히 정의된 degradation이어야 합니다.

### 9.2 Hardware command seam

```
joystick / autonomy command
    → desired sagittal / lateral / yaw
    → DPCBF Filter()
    → filtered command
    → G1 controller
```

무엇이든 연결하기 전에 **raw-command bypass가 없음을 입증해야 합니다.** 즉,
input에서 controller로 이어지는 모든 path가 filter를 통과해야 합니다. 이후
controller input API, command rate, 사용 thread, fail-safe behaviour,
solver-failure behaviour, no-data behaviour, stale-data behaviour, command
timeout, FSM button handling, emergency stop, **dry-run mode**를 명시하십시오.

### 9.3 Hardware time domain

다음 항목이 clock을 공유하는지 확인하십시오: LiDAR packet, IMU packet, DLIO
odometry, ROS system time, ROS steady time, Unitree controller time, DPCBF
query time.

**`frame.stamp`와 `t_query`가 검증된 공통 time domain에 속하기 전에는 staleness
ladder를 안전하게 재사용할 수 없습니다.** Stage 3의 `clock_domain` 측정값이
이를 위한 첫 번째 input입니다.

### 9.4 Hardware shadow mode

Hardware에는 MuJoCo ground truth가 없으므로 simulation의 oracle/shadow ladder를
그대로 적용할 수 없습니다. **Hardware dry-run**을 정의하십시오. DPCBF output을
계산하고 raw command와 filtered command를 log에 기록하되, **filtered output은
절대로 robot에 보내지 않습니다.** External motion capture를 evaluation ground
truth로 선택적으로 사용할 수 있지만 이용 가능하다고 가정해서는 안 됩니다.
