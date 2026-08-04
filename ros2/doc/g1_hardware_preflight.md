# G1 hardware preflight — 첫 session 전에 확인할 정보

**대상: Unitree G1을 처음 사용하는 사람.** 이 문서는 실행 방법을 설명하지
않습니다(실행 방법은
[`g1_first_perception_experiment.md`](g1_first_perception_experiment.md) 참조).
여기서는 **robot의 전원을 켜기 전에 반드시 확인하고 기록해야 할 사항**과 각
답을 확인하는 방법을 설명합니다.

**별도 문서로 분리한 이유.** 이 repository의 모든 수치는 simulation 또는
bench에서 측정한 값입니다. 실제 robot, 실제 Mid-360, 실제 network, 실제 onboard
computer에 대해서는 아무것도 검증되지 않았습니다. Architecture §12
(Deployment)는 vendor documentation을 바탕으로 작성되었으며 명시적으로
미검증 상태입니다. Open question **Q-1**("이 lab의 정확한 G1 variant와 onboard
PC는 무엇인가")은 Phase 0부터 지금까지 미해결 상태입니다. 이 checklist를
작성하기 전까지 hardware path는 deployment가 아니라 *design*일 뿐입니다.

**규칙:** 이 문서를 복사하여
`evidence/hardware/<YYYY-MM-DD>/<session>/preflight.md`에 작성하십시오. 빈칸도
하나의 답입니다. "확인되지 않음"은 정보이지만 추측은 정보가 아닙니다.

**이 phase에서 하지 않는 작업.** Perception-only inference까지만 수행합니다.
DPCBF output은 robot에 전달되지 않고, velocity command를 filtering하지 않으며,
controller도 수정하지 않습니다. 그 이유는 §5를, 이후 phase에 필요한 사항은
첫 번째 experiment runbook의 §9를 참조하십시오.

---

## 1. G1 platform

| # | 항목 | 값 | 확인 방법 |
|---|---|---|---|
| 1.1 | 정확한 G1 variant(EDU / EDU-Ultimate / 기타, 23-dof 또는 29-dof) | | Robot의 sticker, 구매 기록 또는 Unitree app |
| 1.2 | Controller firmware version | | Unitree app → device info 또는 vendor의 `firmware_version` service |
| 1.3 | Robot의 controller software / SDK version | | Onboard PC에서 `ls /unitree/`, `cat /unitree/*/version*` 실행(path는 vendor에 따라 다르므로 실제 확인한 내용을 기록) |
| 1.4 | Onboard computer model | | `cat /proc/device-tree/model`(Jetson) 또는 `sudo dmidecode -s system-product-name`(x86) |
| 1.5 | CPU architecture | | `uname -m` → `x86_64` 또는 `aarch64` |
| 1.6 | Ubuntu version | | `lsb_release -a`, `cat /etc/os-release` |
| 1.7 | 설치된 ROS 2 distribution | | `printenv ROS_DISTRO`, `ls /opt/ros/` |
| 1.8 | RAM | | `free -h` |
| 1.9 | Bag을 저장할 disk의 여유 공간 | | `df -h`, `df -h <bag dir>` |
| 1.10 | sudo/root 사용 가능 여부, Internet 연결 여부 | | `sudo -v`, `ping -c1 deb.debian.org` |
| 1.11 | GPU 존재 및 사용 가능 여부 | | `nvidia-smi` / `tegrastats` |
| 1.12 | 물리적 Ethernet interface(개수, 이름, socket별 대응 관계) | | `ip -br link` 실행 후 cable을 하나씩 분리하며 다시 실행 |
| 1.13 | Wi-Fi interface | | `ip -br link`, `iw dev` |
| 1.14 | Unitree control network를 전달하는 interface | | Vendor documentation + `ip -br addr`, SDK traffic을 관찰하여 확인 |
| 1.15 | Onboard PC를 perception computer로 사용할 것인가? | | **확인이 아니라 결정할 사항** — §3.4 참조 |
| 1.16 | 아니라면 어떤 external workstation을 어떤 방식으로 연결할 것인가? | | |
| 1.17 | Robot이 이미 body odometry 또는 body velocity를 publish하는가? | | SDK domain을 설정한 상태에서 `ros2 topic list \| grep -i -E 'odom\|state'` 실행, Q-5 |
| 1.18 | 물리적 E-stop을 사용할 수 있는가? 위치는 어디인가? | | Robot을 직접 확인하고 lab 담당자에게 문의. **§4 참조** |
| 1.19 | Robot을 tether / suspension / gantry로 지지할 것인가? | | Lab 구성에 따라 확인 |

다음 명령어는 모두 read-only입니다. 실행 결과를 수정하지 말고 record에 그대로
붙여넣으십시오.

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

**Item 1.5는 핵심 조건입니다.** 이 workspace는 aarch64에서 성공적으로 build된
적이 없습니다. Emulated build에서는 **18개 중 10개 package**만 생성되었고,
원인은 분석되었지만 해결되지 않은 upstream `ament_cmake` bug로 중단되었습니다
(`tools/diagnose_ament_export_libraries.py`). Architecture가 `aarch64`라면 robot을
사용하기 **전에** 별도의 build 시간을 확보하고, 먼저 해당 파일과
`tools/build_target.sh`를 읽으십시오. Emulated 결과를 build가 작동한다는
evidence로 간주하지 마십시오.

---

## 2. Mid-360

| # | 항목 | 값 | 확인 방법 |
|---|---|---|---|
| 2.1 | 정확한 model(Mid-360 또는 Mid-360S) | | Sticker |
| 2.2 | Serial number | | Sticker. Livox convention에서는 마지막 두 자리가 IP를 결정함 |
| 2.3 | Firmware version | | Livox Viewer 또는 연결 시 driver의 stdout |
| 2.4 | 현재 LiDAR IP | | Livox Viewer 또는 전원이 켜진 상태에서 `sudo tcpdump -i <iface> -n udp` 실행 |
| 2.5 | LiDAR subnet에서 사용할 host IP | | 직접 결정하되 local interface에 **실제로 할당된** 주소여야 함 |
| 2.6 | Subnet mask | | |
| 2.7 | LiDAR가 연결된 물리적 interface | | 연결 후 `ip -br addr` 실행 |
| 2.8 | Onboard PC에 직접 연결하는가, switch를 거치는가? | | Cabling을 직접 확인 |
| 2.9 | Power source 및 voltage | | |
| 2.10 | 이 network에서 PTP 또는 다른 time sync를 사용할 수 있는가? | | 아래 설명 참조 |
| 2.11 | 실제 mounting position(pelvis 기준 x, y, z) | | 줄자로 측정 — experiment runbook §4 |
| 2.12 | 실제 mounting orientation(roll, pitch, yaw) | | 줄자 측정 + floor-plane 확인 |
| 2.13 | Sensor가 물리적으로 상하 반전되어 있는가? | | Repository에서는 **그렇다고** 가정하며 roll = π(H-1/H-2)임. 실제로 확인할 것 |
| 2.14 | LiDAR→IMU lever arm이 manual과 일치하는가? | | 어떤 unit에서도 검증되지 않음. 아래 설명 참조 |
| 2.15 | Packet loss가 관찰되는가? | | Driver stdout, `ip -s link show <iface>` |

### 2.1 제공된 network config는 그대로 사용할 수 없음

`g1_perception_bringup/config/MID360_config.json`에는 현재 다음과 같은
**upstream sample address**가 들어 있습니다.

```
host_net_info.*_ip : 192.168.1.5
lidar_configs[0].ip: 192.168.1.12
```

이 값은 이 robot의 주소가 아니라 **placeholder**입니다. 파일의 `_comment`
block에도 이 사실이 표시되어 있으며, 해당 값이 남아 있으면
`scripts/g1_hw_preflight.sh`는 exit code **2**로 종료됩니다. 그대로 두면 가장
판단하기 어려운 failure mode가 발생합니다. Driver가 시작되어 `bind failed` /
`Init lds lidar fail!`을 log에 남기지만 **종료되지 않고**, publisher가 lazy이므로
**`/livox/*` topic을 전혀 생성하지 않으며**, **SIGINT와 SIGTERM도 무시합니다.**
따라서 host 설정 오류가 아니라 sensor 고장처럼 보입니다. 이 경우
`pkill -9 -f livo[x]`로 종료하십시오.

두 주소를 모두 교체한 뒤 preflight가 exit code 0으로 종료될 때까지 다시
실행하십시오.

### 2.2 Time synchronisation은 설정값이 아니라 측정값

`MID360_config.json`에는 **timestamp-mode field가 없습니다.** Driver는 각 packet
header에서 sync mode를 읽으며, PTP/GPS source가 없으면 **packet 수신 시점의 host
clock**을 사용합니다. 실제 사용 중인 mode는 runtime에서만 관찰할 수 있습니다.
`scripts/hw_source_probe.py`가 이를 `clock_domain`으로 보고하며, 해당 report가
architecture §14.3의 답이므로 반드시 기록하십시오.

### 2.3 Lever arm은 가정한 constant

`dlio.yaml`과 `t7_hw_extrinsic_guard.py`는 Livox manual의 LiDAR→IMU offset
`(0.011, 0.02329, -0.04412) m`와 identity rotation을 사용합니다. **이 값은 어떤
실제 unit에서도 검증되지 않았습니다.** 약 5 cm의 lever arm이므로 정지 상태에서는
영향이 없고 sway 중 영향도 작습니다. 이 값이 여전히 가정값임을 기록하십시오.

---

## 3. Network topology

### 3.1 세 가지 network

```
              ┌──────────────────────────────────────────────┐
              │             G1 onboard computer              │
              │   (§1의 arch/OS/ROS — 아직 확인되지 않음)    │
              │                                              │
   Unitree    │  ┌────────────┐   ┌──────────┐  ┌─────────┐  │
   control  ──┼──┤ NIC A      │   │ NIC B    │  │ Wi-Fi   │  │
   network    │  │ SDK2 DDS   │   │ Livox    │  │ (dev    │  │
   (rt/…)     │  │ rt/lowstate│   │ UDP      │  │  접속)  │  │
              │  └─────┬──────┘   └────┬─────┘  └────┬────┘  │
              │        │               │             │       │
              │   ┌────┴───────────────┴─────────────┴────┐  │
              │   │  CycloneDDS  (ROS 2 + SDK2, one lib)  │  │
              │   │ CYCLONEDDS_URI로 하나의 NIC에 고정됨  │  │
              │   └───────────────────────────────────────┘  │
              └──────────────────┬───────────────────────────┘
                                 │  (선택 사항)
                        ┌────────┴─────────┐
                        │ dev workstation  │  RViz / recording /
                        │                  │  선택적으로 perception
                        └──────────────────┘
```

### 3.2 Interface 할당 table — **직접 작성할 것**

| Traffic | Protocol | Interface | Address | 비고 |
|---|---|---|---|---|
| Unitree SDK2 / controller | CycloneDDS, `rt/…` topic | | | Domain이 `ROS_DOMAIN_ID`와 일치해야 함 |
| ROS 2 perception topic | CycloneDDS | | | `CYCLONEDDS_URI`로 고정 |
| Livox Mid-360 | raw UDP, port 56100–56501 | | | DDS가 아니므로 domain의 영향을 받지 않음 |
| Dev workstation link | | | | |

**이 repository에 있는 `lo`, `eth0` 또는 그 밖의 interface 이름을 실제 값으로
가정하지 마십시오.** 여기 적힌 interface 이름은 G1에서 확인된 적이 없습니다.
예제에 나오는 `192.168.0.x`/`wlo1` address는 *dev machine*의 값입니다.

### 3.3 하나의 NIC를 함께 사용할 것인가

**하나의** interface가 SDK2 DDS, ROS 2 DDS, Livox UDP를 동시에 전달해야 하는지
명확하게 확인하십시오.

* Livox는 `host_net_info` port로 plain unicast UDP를 전송하므로 다른 traffic과
  함께 사용할 수 있습니다. 단, host가 설정에 지정된 address를 실제로 보유해야
  합니다.
* SDK2와 ROS 2는 설계상 하나의 CycloneDDS instance를 공유합니다(이 workspace가
  하나만 build함, mitigation R-3). 따라서 NIC도 함께 사용할 수 있으며, 이것이
  문서화된 `unitree_ros2` pattern입니다(§12.2).
* Interface가 여러 개라면 **각각의 용도를 명시적으로 할당**하고 ROS/SDK
  traffic을 전달하는 interface에 `CYCLONEDDS_URI`를 고정하십시오. Phase 2에
  기록된 대표적 failure는 Cyclone이 loopback에 bind되는 경우입니다. 이때
  `ros2 topic list`에는 모든 topic 이름이 표시되지만 다른 machine으로는 data가
  전혀 전달되지 않습니다. 바로 이 문제 때문에 preflight script는 loopback
  pin을 hard failure로 처리합니다.

### 3.4 Perception 실행 위치 — session 전에 결정할 것

| | Onboard PC에서 perception 실행 | Workstation에서 perception 실행 |
|---|---|---|
| CPU budget(§17.4, <1 core) | 측정 가능하며 실제 운용 수치임 | **측정 불가** — "Orin benchmark"를 명시적인 후속 항목으로 기록하고, workstation 수치를 onboard budget과 비교하여 보고하지 말 것 |
| Latency(§17.2, p95 ≤ 60 ms) | 실제 운용 수치임 | Network로 인해 증가함 |
| Network | LiDAR + DDS가 local에 있음 | Cloud가 약 3 MB/s로 network를 통과함 |
| Build risk | aarch64 build가 작동해야 함 | Dev-machine build는 이미 작동함 |

결정 사항을 기록하십시오. 이를 정하지 않으면 이후의 모든 측정값은 의미가
없습니다.

---

## 4. Safety 사전 조건 — 문서화 전용이며 타협 불가

이 phase는 **perception-only**입니다.
`g1_perception_hardware_only.launch.py`의 어떤 항목도 command를 publish하지
않으며 `test_hw_offline_gates.py`가 매 build에서 이를 확인합니다. 그렇더라도
robot은 넘어질 수 있는 대형 장비입니다.

다음 조건을 **모두** 만족하고 기록하기 전에는 **walking, command integration,
Stage 12 이후의 절차를 수행하지 마십시오.**

- [ ] **물리적 E-stop**이 있으며 위치를 확인하고 작동을 test함
- [ ] 다른 업무를 수행하지 않는 **전담 E-stop 담당자**를 지정함
- [ ] 최초 motion test에서 robot을 **지지하거나 tether로 고정함**
- [ ] Test 영역을 비움 — robot이 넘어질 수 있는 반경에 사람이 없고 단단한
      모서리가 없음
- [ ] Obstacle prop이 foam/cardboard cylinder처럼 **부드러운 재질**임
- [ ] **Controller recovery procedure**를 숙지하고 소리 내어 읽음
- [ ] **Passive(damping)로 복귀하는 절차**를 숙지함
- [ ] **Safe power-down procedure**를 숙지함
- [ ] ROS 및 perception process를 controller와 **독립적으로** 종료할 수 있음
      (서로 별도의 process임. `pkill -9 -f 'component_containe[r]'` — bracket
      class가 반드시 필요하며, bracket가 없는 pattern은 kill을 실행하는 shell
      자체의 command line과 match됨)
- [ ] Unitree controller의 command stream을 perception과 **독립적으로** 중단할
      수 있음

이 stack에만 해당하는 두 가지 주의 사항:

* **Standing state의 bring-up 순서**(walking 작업에서 확인): robot을 잡은
  상태에서 `L2+up` → FixStand, `R2+A` → RLBase를 실행한 *다음* 내려놓습니다.
  FixStand의 PD hold pose 상태로 내려놓지 **마십시오**.
* Livox driver는 bind에 실패하면 **SIGINT와 SIGTERM을 무시합니다.**
  `ros2 launch`는 15초 후 SIGKILL로 전환하므로 기다리지 마십시오.

---

## 5. 이 phase에서 DPCBF walking까지 진행하지 않는 이유

단순히 조심하기 위한 것이 아닙니다. 다음 네 가지가 아직 존재하지 않습니다.

1. **Hardware `RobotState` source가 없습니다.** DPCBF의 `Filter()`에는
   `{x, y, φ, v_sagittal, v_lateral}`가 필요합니다. Simulation에서는 MuJoCo
   ground truth를 사용했습니다. Hardware에서는 DLIO의 `/odom` pose와 twist,
   Unitree 자체 estimator 또는 두 정보의 fusion을 사용해야 하지만 어느 것도
   아직 구현되지 않았습니다. Walking 중인 G1에서 DLIO의 odometry 품질도
   측정된 적이 없습니다.
2. **Hardware command seam이 없습니다.** `deploy`의 DPCBF insertion point는
   설계되어 있지만(§12.4) 구현되지 않았고, `deploy`는 rclcpp를 link하지 않습니다.
3. **검증된 common time domain이 없습니다.** Staleness ladder는 `frame.stamp`와
   `t_query`를 비교합니다. LiDAR, DLIO, ROS, controller가 clock을 공유하는지는
   위 §2.2의 측정을 완료하기 전까지 알 수 없습니다.
4. **Ground truth가 없습니다.** 이 repository의 모든 accuracy 수치는
   `/sim/gt_obstacles`를 기준으로 평가했습니다. Hardware에는 이에 해당하는
   정보가 없으므로 측량된 fixture를 capture하기 전까지 detector와 safety
   inflation은 정의상 **hardware에서 calibration되지 않은 상태**입니다.

검증되지 않은 odometry와 time domain을 사용하는 calibration되지 않은 safety
filter의 결과를 실제 robot의 velocity command로 전달하는 failure mode를 막기
위해 이러한 순서를 적용합니다.

---

## 6. Hardware에서 calibration되지 않은 parameter

아래 모든 항목은 YAML file에 값이 설정되어 있습니다. 하지만 어떤 값도 hardware
evidence가 아니며, 일부는 잘못된 값임이 확인되었습니다. **Simulation data를
기준으로 tune한 뒤 hardware에서도 유효한 결과라고 제시하지 마십시오.**

| Parameter | File | 상태 |
|---|---|---|
| `measurement_variance: 1.0` | `obstacle_detector.yaml` | **잘못된 값으로 확인됨.** LiDAR measurement의 1σ가 1 m라고 가정하며 모든 track이 σ = 1.0 m로 생성됨. `k_sigma`를 변경하기 전에 hardware data로 산출할 것 |
| `process_variance`, `process_rate_variance` | `obstacle_detector.yaml` | 상속된 값이며 fitting된 적 없음 |
| `fixed_inflation: 0.051` | `safety_obstacle_filter.yaml` | Simulation의 circle-fit bias를 기준으로 **simulation에서 calibration됨** |
| `k_sigma: 2.748`, `sigma_max` | `safety_obstacle_filter.yaml` | 중단된 branch에서 가져온 placeholder, `use_covariance: false` |
| CropBox bound(xy ±0.40, z −0.55…0.45) | `cropbox_self_filter.yaml` | **Simulation 임시값**, *simulated* grounded pose에서 wrist return에 fitting됨 |
| `min_height: 0.15`, `max_height: 1.60`, `range_min: 0.3` | `pointcloud_to_laserscan.yaml` | Appendix-A 값이며 실제 floor에서 확인된 적 없음 |
| `odom/preprocessing/cropBoxFilter/size: 1.0` | `dlio.yaml` | Upstream default, 1 m 이내의 ground도 제거하는 ±1 m cube |
| `odom/geo/K*` observer gain | `dlio.yaml` | Upstream default |
| `tracking_duration`, `min_correspondence_cost` | `obstacle_detector.yaml` | Appendix-A 값이며 simulation에서만 검증됨 |

Simulation에서 유도한 값은 **initial value**로 사용할 수 있습니다. 모든 변경
사항은 근거가 되는 measurement와 함께 기록해야 합니다.

### 6.1 서로 혼동하면 안 되는 네 가지 문제

1. **Self-filtering** — `mid360_link`의 CropBox가 robot-body return을 제거합니다.
2. **Ground rejection** — `base_footprint`의 `min_height`만 사용합니다. **Ground
   segmentation은 없습니다.** Patchwork++는 import, build, launch되지 않으며
   `/points_no_ground`도 존재하지 않습니다. `ground_seg:=patchwork`는 아무
   동작도 하지 않던 no-op이었으며 현재는 명시적인 error를 발생시킵니다. 이
   방식은 **평평한 floor**에서만 작동하며 rough terrain에서의 작동을 보장하지
   않습니다.
3. **Detection/tracking** — extractor가 circle을 fitting하고 tracker가 이를
   smoothing합니다.
4. **Safety inflation** — safety filter가 해당 circle을 확장합니다.

Floor artefact를 숨기기 위해 CropBox를 키우거나 detector error를 숨기기 위해
inflation을 키우면 서로 다른 문제를 혼합하게 되며 양쪽의 evidence를 모두
훼손합니다.

### 6.2 Installed artefact 주의 사항

Launch file, YAML, RViz layout은 **installed artefact**입니다. `ros2 launch`는
사용자가 수정한 file이 아니라 `install/share/g1_perception_bringup/…`을
읽습니다. 어떤 항목이든 수정한 뒤에는 다음을 실행하십시오.

```bash
cd ~/unitree_rl_mjlab_/ros2
colcon build --merge-install --packages-select g1_perception_bringup
source install/setup.bash
```

실제로 사용 중인 copy를 확인하려면 다음을 실행하십시오.

```bash
ros2 run g1_perception_bringup config_diff.py
```

이 명령은 file별 source 및 installed checksum을 출력하고 **stale installed
artefact**를 표시합니다. 이는 source에는 더 이상 존재하지 않아 rebuild 후에도
남아 있는 file입니다. `g1_hw_preflight.sh`는 이를 hard gate로 실행합니다.

---

## 7. Preflight 실행

§1–§3을 모두 작성하고 `MID360_config.json`에 이 robot의 address를 입력한 뒤
다음을 실행하십시오.

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_rl_mjlab_/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<robot의 값>
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml    # ROS NIC에 고정

ros2 run g1_perception_bringup g1_hw_preflight.sh
```

Exit code **0** = Stage 1로 진행합니다. Exit code **1** = hard failure이며, 각
FAIL line에 진행할 수 없는 experiment Stage가 표시됩니다. Exit code **2** =
placeholder network configuration이므로 §2.1로 돌아가십시오.

이 script는 node를 시작하거나 LiDAR socket을 열거나 topic을 publish하지
않습니다. 따라서 robot을 움직일 수 없습니다.

**Preflight 통과가 system의 정상 작동을 의미하지는 않습니다.** 정지된
machine에서 확인할 수 있는 항목만 통과한 것입니다. LiDAR가 data를 생성하는지,
extrinsic이 실제 mount와 일치하는지, odometry가 정상인지는 experiment
runbook의 Stage 3, 4, 5에서 확인합니다.
