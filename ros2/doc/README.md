# G1 DPCBF 실험 — 컴퓨터 구성과 당일 운용 가이드

이 폴더의 **진입 문서**입니다. 실험 당일에는 이 파일만 열어도 되도록,
컴퓨터 3대가 각각 무엇을 하고 / 어디에 무엇을 치고 / 무엇이 정상이고 /
안 되면 무엇부터 보는지를 한 곳에 모았습니다. 세부 절차는 각 문서로
링크하며 여기서 반복하지 않습니다(§8 문서 인덱스).

> **처음부터 세팅하는 경우 — 컴퓨터 2대(Mid-360 직결) 구성:**
> **[`g1_two_computer_setup.md`](g1_two_computer_setup.md)**
> `git clone`부터 apt·빌드·LiDAR IP 탐색·`MID360_config.json` 작성·
> CycloneDDS 연결·실시간 시각화까지 end-to-end 전체 절차입니다.
> 이 파일(3대 구성, `driver:=off`, 워크스페이스 빌드 완료 전제)과 달리
> **Mid-360을 Computer 2에 직접 물리는 2대 구성(`driver:=on`)** 이며
> 빌드 전 상태에서 시작합니다.

---

## 0. 한 장 요약

| | Computer 1 (Blackbox) | Computer 2 (G1 onboard) | Computer 3 (노트북) |
|---|---|---|---|
| 역할 | G1 센서·LiDAR ROS2 토픽 **제공** | perception + DPCBF **계산** | 실시간 plot **표시** |
| 조작 | 건드리지 않음 | Computer 3에서 SSH 접속 | 직접 사용 |
| 실행 | (이미 돌고 있음) | `g1_perception_dpcbf.launch.py` | `dpcbf_plot_client.launch.py` |
| 계산 부하 | 센서 | **전부 여기** | 없음 (표시 전용) |
| 죽으면 | 전체 정지 | 실험 정지 | **아무 영향 없음** |

```
Computer 1 (Blackbox)          Computer 2 (G1 onboard)              Computer 3 (노트북)
─────────────────────          ───────────────────────              ──────────────────
 G1 센서 / Mid-360             perception pipeline
 /livox/lidar  ──── ROS2 ────→   DLIO ──→ /odom ────────┐
 /livox/imu         (센서망)     CropBox → /scan         │
                                 extractor → tracker     │  CycloneDDS
                                 → safety filter         ├──(시각화망)──→ dpcbf_plot_client
                                 → /obstacles_safe ──────┤                 (pyqtgraph GUI)
                                       │                 │
                                       ↓                 │
                                 dpcbf_ros_adapter       │
                                       ↓                 │
                                 DPCBF core (ROS 비종속) │
                                       ↓                 │
                                 /dpcbf/plot  ───────────┘
```

> **Computer 3은 read-only입니다.** subscribe만 하고 로봇이 소비하는 어떤
> 토픽도 publish하지 않습니다. 노트북을 닫든, Wi-Fi가 끊기든, 클라이언트가
> 죽든 Computer 2의 파이프라인과 로봇 제어에는 영향이 없습니다(설계상 —
> 모든 구독이 BestEffort).

---

## 1. 토픽 소유권 — 무엇이 어디서 나오는가

| 토픽 | 타입 | 주기 | 생산자 |
|---|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 10 Hz | **Computer 1** |
| `/livox/imu` | `sensor_msgs/Imu` | 200 Hz | **Computer 1** |
| `/odom` + TF `odom→base_link` | `nav_msgs/Odometry` | 100 Hz | Computer 2 (DLIO) |
| `/points_self_filtered` | `PointCloud2` | 10 Hz | Computer 2 (CropBox) |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | Computer 2 |
| `/raw_obstacles` → `/tracked_obstacles` → `/obstacles_safe` | `obstacle_detector/Obstacles` | 각 10 Hz | Computer 2 |
| `/dpcbf/plot` | `dpcbf_viz_msgs/DpcbfPlotSample` | 30 Hz | Computer 2 (DPCBF seam) |
| `/dpcbf/status`, `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 10 Hz | Computer 2 |

주기는 [`hw_diagnostics.py`](../src/g1_perception/g1_perception_bringup/scripts/hw_diagnostics.py)의
기대값입니다. 절반 이하로 떨어지면 ERROR로 잡힙니다.

### ⚠ 당일 첫 번째로 확인할 것 — LiDAR 토픽 이름

이 저장소의 launch는 DLIO 입력을 `/livox/lidar`, `/livox/imu`로
**고정 remap**합니다([source_hw.launch.py:69-71](../src/g1_perception/g1_perception_bringup/launch/source_hw.launch.py#L69-L71)).
Computer 1이 다른 이름으로 publish하면 DLIO는 조용히 아무것도 받지 못합니다.

```bash
# Computer 2에서, launch 전에
ros2 topic list | grep -i livox
ros2 topic hz /livox/lidar      # 10 Hz
ros2 topic hz /livox/imu        # 200 Hz
```

이름이 다르면 → `source_hw.launch.py`의 remap을 실제 이름으로 고치고
`colcon build --packages-select g1_perception_bringup` 후
`ros2 run g1_perception_bringup config_diff.py`가 PASS인지 확인(설치된
artefact가 바뀌어야 적용됨).

### ⚠ 두 번째 — Mid-360 driver를 Computer 2에서 켜는가

| 상황 | Computer 2 launch 인자 |
|---|---|
| Computer 1이 `/livox/*`를 이미 publish (**현재 구성**) | `driver:=off` |
| Mid-360이 Computer 2에 직접 물려 있음 | `driver:=on` (기본값) |

`driver:=off`는 livox 노드만 띄우지 않고 **DLIO는 그대로 실행**합니다.
`driver:=off`로 운용하면 `MID360_config.json`은 사용되지 않으므로
preflight의 exit 2(placeholder IP)는 그 세션에서 무해합니다 — 단
`driver:=on`이면 그것은 hard stop입니다.

---

## 2. 지금 실제로 되는 것 / 안 되는 것

> **hardware에는 아직 DPCBF control seam이 없습니다.**
> `DpcbfVizPublisher`/`ObstacleSource`를 생성하는 곳은 저장소 전체에서
> [`simulate/src/main.cc`](../../simulate/src/main.cc)(MuJoCo 시뮬레이터)
> 하나뿐이고, 로봇에서 도는 `g1_ctrl`에는 DPCBF 참조가 0건입니다.
> [`g1_first_perception_experiment.md`](g1_first_perception_experiment.md) §9가
> hardware `RobotState` source와 command seam을 아직 **설계 노트**로 두고
> 있는 것과 같은 이야기입니다.

| 실기 세션에서 | 상태 |
|---|---|
| perception 전 구간 (LiDAR → `/obstacles_safe`) | ✅ 동작 |
| Computer 3의 로봇 위치·heading·trail | ✅ `/odom` |
| Computer 3의 장애물 원 + 속도 벡터 | ✅ `/obstacles_safe` |
| Computer 3의 source별 stale 표시 | ✅ |
| `/dpcbf/plot`, nominal/safe command, intervention, min_h | ❌ publisher 없음 → `NO DATA` 표시 |
| 로봇이 DPCBF로 회피 주행 | ❌ 이번 세션 범위 아님 |

즉 이번 실기에서 plot client는 **"노트북에서 보는 실시간 perception 뷰어"**
입니다(SSH 콘솔의 `hw_obstacle_watch.py`가 하던 일의 그래픽 버전).
§9 seam이 붙으면 Computer 3은 **아무 변경 없이** 나머지 절반이 채워집니다 —
이미 그 토픽을 구독하고 있습니다.

**이 세션은 perception 전용이며 어떤 perception 출력도 velocity command로
전달되지 않습니다.** `g1_perception_hardware_only.launch.py` closure에는
actuation 경로가 구조적으로 없고, `hw_offline_gates`가 매 빌드 이를
검사합니다.

---

## 3. 전날 준비 (한 번만)

### Computer 2

> **⚠ Computer 2는 ROS 2 Foxy(Ubuntu 20.04)입니다** — Computer 3(노트북)과
> 개발 머신은 Humble입니다. **워크스페이스 19개 패키지 전부가 Foxy·Humble
> 양쪽에서 빌드**되고(`ros2/tools/foxy_docker.sh build all`, patch 0011–0015),
> C++ 유닛테스트, `hw_offline_gates` 283개 항목(5개 hardware launch가 Foxy
> launch에서 construct되는 것 포함), T10 DDS 공존 게이트가 Foxy에서 PASS합니다.
>
> **Foxy에서 다른 점 한 가지 — CycloneDDS.** Foxy에서는 ROS 미들웨어만
> 배포판 deb(`ros-foxy-rmw-cyclonedds-cpp` 0.7.11)를 씁니다. 소스 조합이
> 존재하지 않기 때문입니다(자세한 이유는 [`../README.md`](../README.md)).
> 런타임에는 그 deb가 unitree_sdk2의 CycloneDDS **0.10.2**를 로드해서
> 정상 동작하며 T10이 이를 확인합니다. 깨지는 건 컴파일 타임 include
> 순서뿐이고, unitree_sdk2와 rclcpp를 한 프로세스에 링크하는 코드는
> unitree의 헤더를 먼저 보게 해야 합니다.
>
> **Foxy↔Humble DDS interop — 검증 완료(2026-08-07).** focal 컨테이너를
> `--network host`로 띄워 호스트 Humble 워크스페이스와 붙인 실측: Computer 3이
> 구독하는 **4개 토픽 전부**가 **양방향·손실 0**으로 전달됩니다
> (`/odom` 1500/1500 @99.99 Hz, `/obstacles_safe` @10.00, `/dpcbf/plot` @30.00),
> multicast·static peers 두 모드 모두. 커스텀 메시지는 필드 단위로 확인했고
> (`CircleObstacle.covariance`, 중첩 `PlotObstacle[]` 포함) 실제
> `dpcbf_plot_client.data_hub`가 라이브로 렌더링합니다. 다만 검증은
> loopback이므로 **실제 두 대 사이의 물리 망**(Wi-Fi multicast, 방화벽)은
> 현장 변수로 남습니다. 상세:
> [`g1_two_computer_setup.md`](g1_two_computer_setup.md) 부록 1.

워크스페이스는 이미 빌드되어 있다고 가정. `~/.g1_viz_env`만 만듭니다.

```bash
ip -br addr        # NIC 이름/IP 확인해서 아래에 적을 것
cat > ~/.g1_viz_env <<'EOF'
export G1_VIZ_DOMAIN_ID=7          # Computer 1/2/3 공통 (로봇 값)
export G1_VIZ_IFACE=wlan0          # Computer 3(노트북)로 나가는 NIC
#export G1_SENSOR_IFACE=eth0       # Computer 1이 다른 NIC일 때만 (dual-NIC XML 자동 선택)
#export G1_VIZ_PEER=192.168.50.30  # multicast 막힌 망일 때 Computer 3 IP
#export G1_SENSOR_PEER=192.168.123.120
EOF
```

### Computer 3
ROS 2 Humble만 있는 상태에서 시작. **6개 패키지**가 검증된 최소 집합입니다
(perception·DLIO·livox·DPCBF 불필요). `rmw_cyclonedds_cpp`가 빠지면
클라이언트가 기동조차 못 합니다 — 이 저장소는 CycloneDDS/rmw를 자체 빌드해
쓰고 `/opt/ros/humble`에는 없습니다.

```bash
sudo apt install -y libarmadillo-dev ros-humble-laser-geometry \
                    python3-pyqtgraph python3-pyqt5
git clone <저장소> ~/unitree_rl_mjlab && cd ~/unitree_rl_mjlab/ros2
./setup_external.sh

source /opt/ros/humble/setup.bash      # ★ 다른 워크스페이스는 source하지 말 것
colcon build --merge-install --packages-select \
    cyclonedds rmw_cyclonedds_cpp obstacle_detector \
    dpcbf_viz_msgs dpcbf_plot_client g1_perception_bringup

cat > ~/.g1_viz_env <<'EOF'
export G1_VIZ_DOMAIN_ID=7          # Computer 2와 동일
export G1_VIZ_IFACE=wlp2s0         # Computer 2로 나가는 NIC
#export G1_VIZ_PEER=192.168.50.20  # static 모드일 때 Computer 2 IP
EOF
```

> 빌드 전에 다른 워크스페이스를 source해 두면 colcon이 그것을 underlay로
> **체인**해서, 그 경로가 없는 실기 환경에서 조용히 깨집니다. Computer 3에서는
> `/opt/ros/humble`만 source한 상태로 빌드하십시오.

---

## 4. 당일 실행 순서

### 4.1 Computer 2 (SSH)

SSH가 끊기면 launch와 bag이 SIGHUP으로 죽습니다. **반드시 tmux 안에서.**

```bash
ssh -o ServerAliveInterval=15 -o ServerAliveCountMax=4 <user>@<onboard-pc>
tmux new -s g1                 # 재접속: tmux attach -t g1
```

**env 블록 — 새 pane마다 다시 붙여넣기.** (새 pane은 tmux server 시작
시점의 환경을 상속하므로 export가 따라오지 않습니다. "토픽 이름은 보이는데
data가 없다"의 최대 원인입니다.)

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_rl_mjlab/ros2/install/setup.bash
source ~/unitree_rl_mjlab/ros2/install/share/g1_perception_bringup/env/viz_env_computer2.sh multicast
#   multicast 안 되는 망: ... viz_env_computer2.sh static   (PEER 변수 필요)
export SESSION=~/unitree_rl_mjlab/ros2/evidence/hardware/$(date +%F)/s1 && mkdir -p "$SESSION"
printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI    # 매 pane 확인
```

| 순서 | pane | 명령 |
|---|---|---|
| 1 | 3 | **입력 확인** — `ros2 topic hz /livox/lidar` (10), `/livox/imu` (200). §1의 이름 확인 |
| 2 | 0 | **preflight** — `ros2 run g1_perception_bringup g1_hw_preflight.sh` |
| 3 | 0 | **스택** — `ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py driver:=off enable_plot_bridge:=true plot_publish_rate:=30.0` |
| 4 | 3 | **출력 확인** — §5 체크포인트 |
| 5 | 1 | **read-out** — `ros2 run g1_perception_bringup hw_obstacle_watch.py` |
| 6 | 2 | **bag** — `ros2 run g1_perception_bringup hw_record.sh` |

`g1_perception_dpcbf.launch.py`는 `g1_perception_hardware_only.launch.py`를
그대로 include하는 superset이라 격리 보장은 동일합니다.
`enable_plot_bridge`/`plot_publish_rate`는 DPCBF control binary가 생기기
전까지 효과가 없습니다(§2).

### 4.2 Computer 3 (노트북)

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_rl_mjlab/ros2/install/setup.bash
source ~/unitree_rl_mjlab/ros2/install/share/g1_perception_bringup/env/viz_env_computer3.sh multicast
#   Computer 2와 같은 모드(multicast|static)를 쓸 것

# 1) GUI 전에 링크부터 — 여기서 안 보이면 GUI를 띄워도 소용없습니다
ros2 topic list | grep -E "odom|obstacles_safe"
ros2 topic hz /odom              # 100 Hz 근처
ros2 topic hz /obstacles_safe    # 10 Hz

# 2) 클라이언트
ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py
#   옵션: window_s:=60.0  gui_rate_hz:=20.0  stale_after_s:=1.0  backend:=matplotlib
```

---

## 5. 체크포인트 — 이 숫자가 나와야 정상

```bash
# Computer 2
ros2 topic hz /livox/lidar            # 10 Hz   (Computer 1)
ros2 topic hz /livox/imu              # 200 Hz  (Computer 1)
ros2 topic hz /odom                   # 100 Hz  (DLIO)
ros2 topic hz /scan                   # 10 Hz
ros2 topic hz /obstacles_safe         # 10 Hz
ros2 topic echo /diagnostics --once   # 전 항목 OK
ros2 run g1_perception_bringup hw_odom_drift.py   # 정지 시 drift 판정

# Computer 3
ros2 topic hz /odom /obstacles_safe   # 위와 같은 값이 보여야 링크 정상
```

Computer 3 화면: 좌상단 배너의 `dpcbf/plot` / `odom` / `obstacles_safe` 세 줄
중 **odom·obstacles_safe가 녹색 `ok`** 이면 정상입니다. `dpcbf/plot`은 §2에
따라 `NO DATA`가 정상입니다. 소스가 끊기면 빨간 `STALE n.ns`로 바뀌고
**GUI는 계속 갱신됩니다**(멈춘 창은 아무것도 말해주지 않으므로 의도한 동작).

---

## 6. 문제 대응

| 증상 | 확인 순서 |
|---|---|
| Computer 2에 `/livox/*`가 없음 | Computer 1이 살아있는지, `ROS_DOMAIN_ID` 일치, 센서망 NIC이 `CYCLONEDDS_URI`에 잡혀 있는지 |
| `/odom`이 안 나옴 | DLIO 입력 토픽 이름(§1) 확인, `/livox/imu` 200 Hz 나오는지, DLIO 로그 |
| `/obstacles_safe`가 비어 있음 | `/scan` 유효 점 개수 → `hw_obstacle_watch.py`로 단계별 확인. `min_height` 밴드는 평평한 바닥 전제 |
| Computer 3 `topic list`가 빔 | 양쪽 `printenv ROS_DOMAIN_ID CYCLONEDDS_URI RMW_IMPLEMENTATION` 일치? pane env 재적용? |
| 이름은 보이는데 data 0 | multicast 차단 → **양쪽** `static` 모드 (`G1_VIZ_PEER` 필요). `ros2 multicast send` / `receive`로 판정 |
| Computer 2에서만 안 나감 | 센서망/시각화망이 다른 NIC → `G1_SENSOR_IFACE` 설정. **`CYCLONEDDS_URI`는 프로세스당 하나이며 기존 `~/cyclonedds.xml`과 병합되지 않습니다** |
| Computer 3 클라이언트 즉사 | `librmw_cyclonedds_cpp.so` 없음 → §3의 6개 패키지 재확인 |
| 장애물 원만 안 보임 | `obstacle_detector` 미빌드 (클라이언트 로그에 그렇게 찍힙니다) |
| 방화벽 의심 | UDP 7400–7500 (도메인에 따라 이동) |

---

## 7. 절대 규칙

1. **bag 없는 stage는 실패한 stage입니다.** 로봇 시간에 debug하지 말고
   capture하십시오. 판단 근거가 되는 출력은 전부 `$SESSION`에 파일로.
2. **tmux 밖에서 launch/record 금지.** SSH 끊김 = bag 잘림.
3. **모든 pane에서 env 재적용.** 특히 `ROS_DOMAIN_ID`, `CYCLONEDDS_URI`.
4. **hardware에서 `use_sim_time`은 어디서나 false**이며 launch 인자도
   아닙니다(`/clock`이 없으므로 true인 노드는 타이머가 영영 안 돕니다).
5. **config는 설치된 artefact입니다.** YAML/launch/RViz 수정 후에는
   `colcon build --packages-select g1_perception_bringup` →
   `ros2 run g1_perception_bringup config_diff.py`가 PASS여야 적용됩니다.
6. **Computer 3은 표시 전용.** 로봇 command를 publish하지 않으며, 죽어도
   Computer 2에 영향을 주지 않습니다.
7. **이 세션에 actuation은 없습니다.** perception 출력이 velocity command로
   가는 경로 자체가 존재하지 않습니다.

---

## 8. 문서 인덱스 — 언제 무엇을 여는가

| 상황 | 문서 |
|---|---|
| **아무것도 없는 상태에서 컴퓨터 2대 full 세팅 (clone→빌드→LiDAR IP→시각화)** | [`g1_two_computer_setup.md`](g1_two_computer_setup.md) |
| 로봇 전원 켜기 전에 알아야 할 것 | [`g1_hardware_preflight.md`](g1_hardware_preflight.md) |
| 검증된 것 / 검증 안 된 것 구분 | [`g1_hardware_code_audit.md`](g1_hardware_code_audit.md) |
| **perception 세션 stage별 정확한 절차** | [`g1_first_perception_experiment.md`](g1_first_perception_experiment.md) |
| 현장 도착~철수 전체 흐름 (영문) | [`g1_first_day_field_runbook.md`](g1_first_day_field_runbook.md) |
| **plot 링크 상세 (토픽/QoS/CycloneDDS/검증)** | [`dpcbf_plot_visualization.md`](dpcbf_plot_visualization.md) |
| 스택 실행 일반·시뮬레이션·트러블슈팅 | [`operator_runbook.md`](operator_runbook.md) |
| capture 계획 (블록 구조) | [`phase5b_checklists.md`](phase5b_checklists.md) |
| 파이프라인 내부 동작·수치 근거 | [`pipeline_technical_report.md`](pipeline_technical_report.md) |
| 워크스페이스 구성·pin·patch | [`../README.md`](../README.md) |

---

## 9. 세션 종료

```bash
# Computer 3: 클라이언트 Ctrl-C (아무 영향 없음)
# Computer 2: bag pane Ctrl-C → 스택 pane Ctrl-C 순서
du -sh "$SESSION"        # bag 크기 기록
```

**메타데이터 마무리 — 로봇이 아직 눈앞에 있을 때.** `hw_record.sh`가
기계가 알 수 있는 정보(commit, 환경, 로드된 YAML의 checksum, network)는
이미 `<bag>.session.json`에 기록했습니다. 사람만 아는 필드는 비어 있고,
스크립트는 그것을 **exit 1 + 빈 필드 목록**으로 알려줍니다.

```bash
BAG=<bag 경로>
ros2 run g1_perception_bringup hw_session_metadata.py \
    --out "$BAG.session.json" --bag "$BAG" --copy-configs \
    --g1-variant "G1 EDU" --lidar-serial <serial> --operator "<이름>" \
    --robot-state Passive --scenario "<프롭 배치 설명>"
#   exit 0 = metadata complete. exit 1이면 출력된 빈 필드를 채워 다시 실행
```

bag과 `$SESSION`을 Computer 3으로 복사한 뒤 offline 분석을 진행하십시오
(현장에서 분석하지 않습니다). provenance를 나중에 재구성한 bag은 evidence가
아닙니다.
