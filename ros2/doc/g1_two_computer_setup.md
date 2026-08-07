# G1 실기 실험 — 컴퓨터 2대 End-to-End 세팅 가이드

**목표**: 아무것도 설치되지 않은 상태에서 시작해서, `git clone` → 빌드 →
Mid-360 LiDAR 연결 → DLIO 오도메트리 → 장애물 검출·추적 → **Computer 3
노트북에서 실시간 시각화**까지 한 번에 끝내는 것.

**시나리오**: 로봇은 **정지 상태**로 서 있고, 사람이나 물체가 로봇 앞을
지나가면 그 물체의 **위치·반지름·속도**가 실시간으로 추정되어 노트북
화면에 그려진다.

이 문서는 **처음 하는 사람이 이 파일 하나만 보고 끝까지 갈 수 있도록**
쓰였습니다. 모든 명령에는 **어느 컴퓨터에서 / 어느 디렉토리에서 / 몇 번
터미널에서** 실행하는지가 붙어 있습니다.

> 기존 [`README.md`](README.md)(3대 구성: Blackbox가 `/livox/*`를 주는 구성)와
> 달리, 이 문서는 **Mid-360을 Computer 2에 직접 물리는 2대 구성**입니다.
> 즉 `driver:=on`이고, `MID360_config.json`을 **반드시 채워야** 합니다.

---

## 목차

- [0. 구성 개요와 전제](#0-구성-개요와-전제)
- [Part A — Computer 2 (G1 온보드, ROS 2 Foxy)](#part-a--computer-2-g1-온보드-ros-2-foxy)
  - [A-1. 사전 확인](#a-1-사전-확인-터미널-1개)
  - [A-2. apt 의존성 설치](#a-2-apt-의존성-설치)
  - [A-3. 저장소 clone](#a-3-저장소-clone)
  - [A-4. 외부 소스 import + 패치](#a-4-외부-소스-import--패치-setup_externalsh)
  - [A-5. 빌드](#a-5-빌드-foxy)
  - [A-6. 빌드 검증](#a-6-빌드-검증)
  - [A-7. Mid-360 LiDAR IP 찾기](#a-7-mid-360-lidar-ip-찾기)
  - [A-8. MID360_config.json 채우기](#a-8-mid360_configjson-채우기)
  - [A-9. 네트워크 환경파일 `~/.g1_viz_env`](#a-9-네트워크-환경파일-g1_viz_env)
  - [A-10. Preflight](#a-10-preflight--실행-전-마지막-관문)
  - [A-11. 단계별 기동 검증 (3단계)](#a-11-단계별-기동-검증-3단계)
  - [A-12. Foxy 전용 주의사항](#a-12-foxy-전용-주의사항-반드시-읽을-것)
- [Part B — Computer 3 (노트북, ROS 2 Humble)](#part-b--computer-3-노트북-ros-2-humble)
- [Part C — 두 컴퓨터 연결 (CycloneDDS)](#part-c--두-컴퓨터-연결-cyclonedds)
- [Part D — 실험 실행 시트 (복붙용)](#part-d--실험-실행-시트-복붙용)
- [Part E — 문제 해결](#part-e--문제-해결)
- [Part F — 세션 종료와 기록](#part-f--세션-종료와-기록)
- [부록 1. 검증 로그](#부록-1-검증-로그--이-문서의-어디까지가-실제로-확인된-것인가)
- [부록 2. 전체 파이프라인 데이터 흐름](#부록-2-전체-파이프라인-데이터-흐름)

---

## 0. 구성 개요와 전제

### 0.1 컴퓨터 2대

| | **Computer 2** | **Computer 3** |
|---|---|---|
| 정체 | G1 온보드 PC (로봇 안) | 로컬 우분투 노트북 |
| OS / ROS | **Ubuntu 20.04 / ROS 2 Foxy** | Ubuntu 22.04 / ROS 2 Humble |
| 연결 | Mid-360 LiDAR 직결 + Wi-Fi/이더넷 | Wi-Fi/이더넷 |
| 역할 | 센서 드라이버 + 오도메트리 + 장애물 검출·추적 **전부** | 실시간 **표시 전용** |
| 접속 방법 | Computer 3에서 SSH | 직접 사용 |
| 계산 부하 | 전부 여기 | 거의 없음 |
| 죽으면 | 실험 정지 | **아무 영향 없음** |

### 0.2 데이터 흐름 (한 줄 요약)

```
Mid-360 ──UDP──> [C2] livox 드라이버 ──/livox/lidar,/livox/imu──> DLIO ──/odom + TF──┐
                                              │                                      │
                                              └──> CropBox ──> /scan ──> extractor ──> tracker ──> safety filter
                                                                                                        │
                                                                                              /obstacles_safe
                                                                                                        │
                                                        ══════ CycloneDDS (Wi-Fi) ══════════════════════┤
                                                                                                        ▼
                                                                                    [C3] dpcbf_plot_client (GUI)
```

### 0.3 이 세션에서 되는 것 / 안 되는 것

| 항목 | 상태 |
|---|---|
| LiDAR → 오도메트리 → 장애물 원 + 속도 추정 | ✅ 전부 동작 |
| 노트북에서 로봇 위치·heading·trail 실시간 표시 | ✅ (`/odom`) |
| 노트북에서 장애물 원 + 속도 벡터 실시간 표시 | ✅ (`/obstacles_safe`) |
| 노트북 화면의 `dpcbf/plot` 줄 | ❌ **`NO DATA`가 정상** (아래 참조) |
| nominal/safe command, intervention, min_h 그래프 | ❌ 빈 화면 (정상) |
| 로봇이 장애물을 피해 걷는 것 | ❌ 이번 세션 범위 아님 |

> **`/dpcbf/plot`이 비는 이유**: DPCBF control seam(`DpcbfVizPublisher`)을
> 생성하는 곳은 저장소 전체에서 MuJoCo 시뮬레이터
> [`simulate/src/main.cc`](../../simulate/src/main.cc) 하나뿐이고, 로봇에서 도는
> `g1_ctrl`에는 DPCBF 참조가 0건입니다. 따라서 실기에서 이 클라이언트는
> **"노트북에서 보는 실시간 perception 뷰어"** 입니다. seam이 붙으면
> Computer 3은 **아무 변경 없이** 나머지 절반이 채워집니다 — 이미 그 토픽을
> 구독하고 있습니다.

### 0.4 안전 보증

- 이 세션에는 **actuation 경로가 구조적으로 존재하지 않습니다.** perception
  출력이 velocity command로 가는 코드가 없고, `hw_offline_gates` 283개 검사가
  매 빌드마다 이를 확인합니다 (Foxy에서 283/283 PASS 확인함 — 부록 1).
- Computer 3은 **subscribe 전용**입니다. 노트북을 닫든 Wi-Fi가 끊기든
  Computer 2의 파이프라인에 영향이 없습니다 (모든 구독이 BestEffort).

### 0.5 준비물 체크리스트

- [ ] G1 로봇 + Mid-360 (토르소 상단, 뒤집힌 방향 마운트)
- [ ] Computer 2에 sudo 권한이 있는 계정
- [ ] Computer 2 여유 디스크 **20 GB 이상** (bag 녹화 시 ~3 MB/s)
- [ ] 두 컴퓨터가 같은 네트워크 (Wi-Fi 또는 유선)
- [ ] **Mid-360 본체 스티커의 시리얼 번호** (IP 추정에 필요)
- [ ] 인터넷 (clone + apt + vcs import)

---

# Part A — Computer 2 (G1 온보드, ROS 2 Foxy)

> **Part A는 터미널 1개**로 전부 진행합니다 (A-11 기동 검증부터 여러 개).
> Computer 3에서 SSH로 붙어서 작업하는 경우, **A-3부터는 tmux 안에서**
> 하십시오. 빌드 중 SSH가 끊기면 빌드가 SIGHUP으로 죽습니다.

```bash
# [Computer 3에서 실행] SSH 접속
ssh -o ServerAliveInterval=15 -o ServerAliveCountMax=4 <user>@<onboard-pc-ip>

# [Computer 2] 접속 직후 tmux 시작 (끊겨도 살아남음)
tmux new -s g1
#   재접속:  tmux attach -t g1
#   pane 분할: Ctrl-b "  (가로)  /  Ctrl-b %  (세로)  /  Ctrl-b 방향키 (이동)
```

---

## A-1. 사전 확인 (터미널 1개)

**실행 위치: 아무 데나 (홈 디렉토리)**

```bash
cd ~
lsb_release -a                    # Ubuntu 20.04.x LTS (focal) 이어야 함
ls /opt/ros/                      # foxy 가 보여야 함
source /opt/ros/foxy/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"     # foxy
nproc && free -h && df -h ~       # 코어 / 메모리 / 디스크(20 GB 이상)
ip -br addr                       # ★ NIC 이름과 IP를 메모해 두십시오
```

`ip -br addr` 출력 예시 — **이 두 줄을 적어 두어야 합니다**:

```
lo               UNKNOWN        127.0.0.1/8
eth0             UP             192.168.1.50/24        <- LiDAR 쪽 NIC (예)
wlan0            UP             192.168.50.20/24       <- 노트북 쪽 NIC (예)
```

| 적어둘 것 | 예시 값 | 나중에 쓰는 곳 |
|---|---|---|
| LiDAR 쪽 NIC 이름 | `eth0` | `G1_SENSOR_IFACE` |
| LiDAR 쪽 NIC IP | `192.168.1.50` | `MID360_config.json`의 `host_net_info` |
| 노트북 쪽 NIC 이름 | `wlan0` | `G1_VIZ_IFACE` |
| 노트북 쪽 NIC IP | `192.168.50.20` | Computer 3의 `G1_VIZ_PEER` |

> **NIC이 하나뿐이라면** (LiDAR와 노트북이 같은 망) — 더 간단합니다.
> `G1_SENSOR_IFACE`는 설정하지 않고 `G1_VIZ_IFACE` 하나만 씁니다.

> ⚠ **기존 Unitree 환경 설정 확인.** G1 온보드 PC에는 이미 Unitree SDK용
> 설정이 `~/.bashrc`에 들어 있을 수 있습니다. 확인하십시오:
> ```bash
> grep -nE "ROS_DOMAIN_ID|CYCLONEDDS_URI|RMW_IMPLEMENTATION|setup.bash" ~/.bashrc
> ```
> 무언가 나오면 **지우지 말고 기록만** 해 두십시오. 이 문서의 절차는
> `CYCLONEDDS_URI`를 **덮어씁니다** — CycloneDDS는 프로세스당 XML 하나만
> 읽고 두 XML을 병합하지 않기 때문입니다. 기존 설정이 필요한 다른 작업은
> 이 세션과 같은 셸에서 하지 마십시오.

---

## A-2. apt 의존성 설치

**실행 위치: 아무 데나**

아래 목록은 이 워크스페이스 `package.xml` 전체의 의존성을 Foxy/focal deb
이름으로 해석한 것입니다 (`ros2/docker/foxy/Dockerfile`과 동일 = Foxy 빌드가
실제로 통과한 조합).

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    build-essential cmake git tmux \
    net-tools iputils-ping tcpdump iproute2 \
    python3-pip python3-colcon-common-extensions python3-vcstool \
    python3-pytest python3-pytest-cov python3-yaml python3-numpy \
    libarmadillo-dev libpcl-dev libssl-dev libcunit1-dev \
    libyaml-cpp-dev libeigen3-dev libfmt-dev libomp-dev \
    libpcap-dev libapr1-dev \
    ros-foxy-ament-cmake ros-foxy-ament-cmake-auto ros-foxy-ament-cmake-gtest \
    ros-foxy-ament-lint-auto ros-foxy-ament-lint-common \
    ros-foxy-diagnostic-msgs ros-foxy-diagnostic-updater \
    ros-foxy-geometry-msgs ros-foxy-laser-geometry \
    ros-foxy-launch ros-foxy-launch-ros ros-foxy-launch-testing \
    ros-foxy-launch-testing-ament-cmake ros-foxy-launch-testing-ros \
    ros-foxy-message-filters ros-foxy-nav-msgs \
    ros-foxy-pcl-conversions ros-foxy-pcl-msgs \
    ros-foxy-rclcpp ros-foxy-rclcpp-components ros-foxy-rclpy \
    ros-foxy-rmw ros-foxy-rmw-dds-common ros-foxy-rmw-implementation \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-robot-state-publisher \
    ros-foxy-rosbag2 ros-foxy-rosbag2-storage-default-plugins \
    ros-foxy-rosidl-default-generators ros-foxy-rosidl-default-runtime \
    ros-foxy-sensor-msgs ros-foxy-std-msgs ros-foxy-std-srvs \
    ros-foxy-tf2 ros-foxy-tf2-eigen ros-foxy-tf2-geometry-msgs \
    ros-foxy-tf2-ros ros-foxy-tf2-sensor-msgs \
    ros-foxy-visualization-msgs ros-foxy-xacro
```

**선택 사항** — Computer 2에 모니터가 붙어 있고 RViz나 plot GUI를 온보드에서
직접 띄우고 싶을 때만 (Part C의 fallback에서 씁니다):

```bash
sudo apt-get install -y ros-foxy-rviz2 \
    python3-pyqtgraph python3-pyqt5 python3-pyqt5.qtopengl python3-matplotlib
```

> **`ros-foxy-rmw-cyclonedds-cpp`는 반드시 deb로 깝니다.** 이 저장소는
> Humble에서는 `rmw_cyclonedds_cpp`를 소스로 빌드하지만 Foxy에서는 그럴 수
> 없습니다: foxy 브랜치는 CycloneDDS **0.7**의 `ddsi_sertopic` API로
> 작성되어 있는데 이 워크스페이스는 unitree_sdk2 때문에 **0.10.2**를 핀하고,
> humble 브랜치는 Foxy에 없는 rmw 헤더를 요구합니다. 양쪽을 만족하는 소스
> 조합이 존재하지 않습니다. 런타임에는 이 0.7 빌드 rmw가 `~/cyclonedds_ws`
> underlay의 0.10.2 `libddsc.so.0`을 로드해서 정상 동작합니다 (0.10.2가 구
> ABI를 유지했고, `t10_dds_coexistence` 게이트가 Foxy에서 이를 확인합니다).
> 같은 이유로 CycloneDDS 0.10.2 자체도 워크스페이스에서 빌드하지 않고 이
> underlay 것을 씁니다 — A-5의 `--packages-skip` 참조.

> **Foxy는 EOL입니다.** `packages.ros.org`에서 내려갔지만
> `snapshots.ros.org/foxy/final` 아카이브에는 위 deb가 전부 남아 있습니다.
> `apt-get update`가 404를 뱉으면 `/etc/apt/sources.list.d/ros2*.list`를
> 확인하고, 필요하면 아래로 교체하십시오. **다른 방식으로 "고치지"
> 마십시오.**
> ```bash
> echo "deb [arch=$(dpkg --print-architecture)] http://snapshots.ros.org/foxy/final/ubuntu focal main" \
>   | sudo tee /etc/apt/sources.list.d/ros2.list
> sudo apt-get update
> ```

---

## A-3. 저장소 clone

**실행 위치: 홈 디렉토리**

```bash
cd ~
git clone https://github.com/PSH0823/unitree_rl_mjlab.git
cd ~/unitree_rl_mjlab
git checkout obstacle_detection
git log --oneline -1        # 어떤 커밋인지 기록해 두십시오
```

정상 출력 예:
```
8de8c18 Humble + computer 3 visualization
```

> 저장소 경로를 `~/unitree_rl_mjlab`로 두는 것을 **강력히 권장**합니다. 이
> 문서와 다른 모든 문서의 경로가 그 기준입니다. 다른 곳에 두면 이하 모든
> `~/unitree_rl_mjlab`를 본인 경로로 바꿔 읽으십시오.

---

## A-4. 외부 소스 import + 패치 (`setup_external.sh`)

**실행 위치: `~/unitree_rl_mjlab/ros2`**

이 스크립트는 `deps.repos`에 **SHA로 핀된** 외부 저장소 12개를
`src/external/`로 가져오고, 기록된 패치 0001–0015를 적용합니다. Foxy
포팅 패치(0011–0015)가 여기서 들어갑니다.

```bash
cd ~/unitree_rl_mjlab/ros2
./setup_external.sh
```

정상 출력의 마지막 줄:
```
External sources ready.
```

> 두 번째부터 실행하면 각 패치가 `... patch already applied`로 표시됩니다.
> 이 스크립트는 **idempotent**하므로 여러 번 실행해도 안전합니다.

가져온 것 확인:

```bash
cd ~/unitree_rl_mjlab/ros2
ls src/external
```
```
Livox-SDK2  MuJoCo-LiDAR  cyclonedds  cyclonedds-cxx
direct_lidar_inertial_odometry  livox_ros_driver2  obstacle_detector_2
perception_pcl  pointcloud_to_laserscan  rmw_cyclonedds  unitree_dds_wrapper
unitree_sdk2
```

**핵심 외부 패키지 4개**가 무엇을 하는지:

| 패키지 | 역할 |
|---|---|
| `Livox-SDK2` (v1.3.1) | Mid-360 UDP 프로토콜. `package.xml`이 없어 COLCON_IGNORE되고 `livox_sdk2_vendor`가 빌드해 넣습니다 |
| `livox_ros_driver2` (1.2.6) | `/livox/lidar` (PointCloud2) + `/livox/imu` publish |
| `direct_lidar_inertial_odometry` (DLIO) | LiDAR-관성 오도메트리 → `/odom` + TF `odom→base_link` |
| `obstacle_detector_2` | scan → 원(circle) 검출 → Kalman 추적 (패치 P-1~P-5로 대폭 수정됨) |

---

## A-5. 빌드 (Foxy)

**실행 위치: `~/unitree_rl_mjlab/ros2`**

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash

colcon build \
    --packages-skip rmw_cyclonedds_cpp cyclonedds \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

**세 가지를 반드시 지켜야 합니다:**

1. **`--packages-skip rmw_cyclonedds_cpp cyclonedds`** — 빼면 빌드가
   실패합니다. `rmw_cyclonedds_cpp`는 Foxy에서 deb를 씁니다 (A-2의 설명).
   `cyclonedds`는 Computer 2의 `~/cyclonedds_ws` underlay가 **동일한 핀
   버전 0.10.2**를 이미 제공하므로 워크스페이스에서 다시 빌드하지 않습니다.
   빌드를 시도하면 코드 생성 단계의 `idlc`가 LD_LIBRARY_PATH에 있는
   `/opt/ros/foxy`의 CycloneDDS **0.7** `libddsc`를 먼저 로드해서
   `undefined symbol: DDS_XTypes_TypeObject_desc`로 죽습니다
   (2026-08-07 Computer 2에서 실측).
2. **`--merge-install`을 쓰지 않습니다.** Foxy에서 실제로 검증된 레이아웃은
   기본(isolated) 레이아웃입니다. (Humble 개발 머신의 `ros2/README.md`
   명령에는 `--merge-install`이 있는데, 그건 개발 머신 쪽 규약입니다.)
3. **다른 워크스페이스를 source한 상태로 빌드하지 마십시오.** colcon이 그걸
   underlay로 체인해서, 그 경로가 없는 환경에서 조용히 깨집니다.
   **예외는 `~/cyclonedds_ws` 하나뿐입니다** — Computer 2의 `~/.bashrc`가
   이를 source하며, 위 1의 `cyclonedds` 스킵이 바로 이 underlay를 전제로
   합니다.

**소요 시간**: 온보드 PC에서 **40–90분**. 메모리가 부족해 죽으면
`--parallel-workers 2`를 추가하십시오.

정상 종료 시 마지막 줄:
```
Summary: 18 packages finished [xx min yy s]
```

빌드되는 18개 패키지 (`cyclonedds`는 underlay에서 오므로 목록에 없습니다):

```
unitree_sdk2                    unitree_dds_wrapper_vendor livox_sdk2_vendor
livox_ros_driver2               direct_lidar_inertial_odometry
pcl_ros                         pointcloud_to_laserscan    obstacle_detector
safety_obstacle_filter          g1_perception_utils        g1_description
g1_perception_bringup           dpcbf_viz_msgs             dpcbf_ros_adapter
dpcbf_plot_client               sim_msgs                   sim_mjlidar_bridge
t10_dds_coexistence
```

> **빌드가 중간에 실패하면** — 어떤 패키지에서 멈췄는지 확인하고
> `log/latest_build/<패키지>/stdout_stderr.log`를 보십시오. 자주 나오는
> 원인 5가지와 각각이 막는 에러 메시지는
> [`../tools/build_target.sh`](../tools/build_target.sh) 헤더에 정리되어
> 있습니다 (VTK imported target, OpenSSL, PCL/Eigen, livox 빌드 순서, ament
> `_lib` 캐시 섀도우).

---

## A-6. 빌드 검증

**실행 위치: `~/unitree_rl_mjlab/ros2`**

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash          # ★ 이제부터 모든 셸에서 이 두 줄이 필요합니다

# 1) 실행 파일이 다 있는가 (Foxy의 ros2 pkg executables 는 인자 1개만 받습니다)
ros2 pkg executables livox_ros_driver2
ros2 pkg executables direct_lidar_inertial_odometry
ros2 pkg executables g1_perception_utils
ros2 pkg executables g1_perception_bringup

# 2) CropBox 컴포넌트 라이브러리가 설치됐는가
find install -name libpcl_ros_filters.so
```

기대 출력:

```
livox_ros_driver2 livox_ros_driver2_node
direct_lidar_inertial_odometry dlio_odom_node
direct_lidar_inertial_odometry dlio_map_node
g1_perception_utils base_footprint_publisher
g1_perception_utils dpcbf_overlay_node
g1_perception_utils obstacles_marker_relay
```
`g1_perception_bringup`은 **13개**가 나옵니다:
```
g1_perception_bringup config_diff.py
g1_perception_bringup g1_hw_preflight.sh
g1_perception_bringup hw_config_check.py
g1_perception_bringup hw_diagnostics.py
g1_perception_bringup hw_obstacle_watch.py
g1_perception_bringup hw_odom_drift.py
g1_perception_bringup hw_probe_core.py
g1_perception_bringup hw_record.sh
g1_perception_bringup hw_session_metadata.py
g1_perception_bringup hw_source_probe.py
g1_perception_bringup hw_source_stub.py
g1_perception_bringup hw_tf_probe.py
g1_perception_bringup selfhit_analysis.py
```
그리고 `find`가 최소 한 줄:
```
install/pcl_ros/lib/libpcl_ros_filters.so
```

**하나라도 빠지면 빌드가 완료되지 않은 것입니다. 다음으로 넘어가지
마십시오.**

---

## A-7. Mid-360 LiDAR IP 찾기

**실행 위치: `~/unitree_rl_mjlab/ros2` (어디든 무방)**

LiDAR IP를 모르면 드라이버는 **에러 없이 조용히 아무 토픽도 만들지
않습니다.** 이것이 이 단계가 별도로 있는 이유입니다.

### 알아야 할 사실

- Livox SDK2는 시작할 때 **1초마다** `255.255.255.255:56000`으로
  `LidarSearch` 브로드캐스트를 보내고, LiDAR는 **자기 IP에서** 응답합니다
  (`Livox-SDK2/sdk_core/device_manager.cpp` `Detection()`).
  → **UDP 56000 포트를 스니핑하면 LiDAR IP가 그대로 보입니다.**
- 단, 그 브로드캐스트를 보내려면 SDK가 먼저 `host_net_info`의 IP에 소켓을
  bind해야 합니다. 그래서 **호스트 IP부터 맞추고 → 드라이버를 띄우고 →
  스니핑**하는 순서가 됩니다.
- Livox 공장 관례: **`192.168.1.1XX`**, `XX` = 시리얼 번호 **마지막 두 자리**.
  단 G1에 통합 출고된 유닛은 로봇 내부망(`192.168.123.x` 등)으로 재설정되어
  있을 수 있으므로 **관례는 첫 번째 추측일 뿐** 확인이 필요합니다.

### 방법 1 — 시리얼 스티커 (가장 빠름)

Mid-360 본체 스티커의 시리얼 번호를 읽습니다. 예: `...ABCD**47**` →
추정 IP `192.168.1.147`.

```bash
ping -c 3 192.168.1.147
```
응답이 오면 확정입니다. (일부 유닛은 ICMP에 응답하지 않도록 설정되어
있으므로, 무응답이 곧 오답은 아닙니다 — 방법 3으로 확인하십시오.)

### 방법 2 — 수동 스캔 (같은 서브넷일 때)

```bash
# LiDAR 쪽 NIC의 IP/서브넷을 먼저 확인
ip -br addr show eth0                 # 예: 192.168.1.50/24

# ARP 테이블 (이미 통신한 적 있으면 여기 남아 있습니다)
ip neigh show dev eth0

# 서브넷 전체 스캔
sudo apt-get install -y nmap
sudo nmap -sn 192.168.1.0/24
```

Livox 유닛은 보통 MAC OUI가 `Livox`로 표시됩니다.

### 방법 3 — 패시브 스니핑 (가장 확실)

LiDAR는 마지막으로 설정된 호스트로 데이터를 계속 쏘고 있을 수 있습니다.

```bash
# 터미널 A — LiDAR 쪽 NIC에서 UDP 관찰
sudo tcpdump -ni eth0 udp -c 30
```

출력에서 **56000/56100/56200/56300/56400 포트**가 관련된 줄의 **source IP**가
LiDAR입니다:
```
IP 192.168.1.147.56000 > 192.168.1.50.56000: UDP, length 60
        ^^^^^^^^^^^^^  <- 이것이 LiDAR IP
```

아무것도 안 잡히면 **능동 탐색**으로 넘어갑니다:

```bash
# 터미널 A — 스니퍼를 먼저 켜 둠
sudo tcpdump -ni eth0 udp port 56000

# 터미널 B — 호스트 IP를 A-8 절차대로 임시로 채운 뒤 드라이버만 기동
#            (LiDAR IP는 아직 틀려도 됩니다 — 브로드캐스트는 나갑니다)
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash && source install/setup.bash
ros2 launch g1_perception_bringup source_hw.launch.py driver:=on lio:=off
```
터미널 A에 1초마다 브로드캐스트가 나가고, 응답하는 LiDAR의 source IP가
찍힙니다. 확인 후 터미널 B는 `Ctrl-C`.

### 방법 4 — 호스트 IP가 LiDAR 서브넷에 없을 때

LiDAR가 공장 기본값(`192.168.1.1XX`)인데 온보드 NIC이 다른 대역이면,
NIC에 임시 IP 별칭을 붙여 같은 서브넷으로 들어갑니다:

```bash
sudo ip addr add 192.168.1.5/24 dev eth0     # 임시 추가
ip -br addr show eth0                        # 두 IP가 다 보이는지 확인
# ... 탐색 후 필요 없으면
sudo ip addr del 192.168.1.5/24 dev eth0
```

> ⚠ 로봇 내부망 NIC의 기존 IP는 **절대 지우지 마십시오** (`ip addr del`을
> 기존 주소에 쓰지 말 것). Unitree 통신이 끊깁니다. **추가만** 하십시오.

### 결과 기록

| 항목 | 값 (여기에 적으십시오) |
|---|---|
| LiDAR IP | `192.168.1.___` |
| 호스트(Computer 2) LiDAR 쪽 IP | `192.168.1.___` |
| LiDAR 쪽 NIC 이름 | `eth0` |
| Mid-360 시리얼 | |

두 IP는 **같은 /24 서브넷**이어야 합니다 (아니면 `hw_config_check.py`가
경고합니다).

---

## A-8. `MID360_config.json` 채우기

**실행 위치: `~/unitree_rl_mjlab/ros2`**

파일 경로 (원본 = 소스 트리, 여기를 고칩니다):

```
~/unitree_rl_mjlab/ros2/src/g1_perception/g1_perception_bringup/config/MID360_config.json
```

```bash
cd ~/unitree_rl_mjlab/ros2
nano src/g1_perception/g1_perception_bringup/config/MID360_config.json
```

### 고칠 곳 — 정확히 6개 IP + 1개 IP

```jsonc
  "MID360": {
    "lidar_net_info": {          // ← 손대지 마십시오 (LiDAR 쪽 포트, 고정값)
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip":   "192.168.1.50",   // ★ Computer 2의 LiDAR 쪽 NIC IP
      "cmd_data_port": 56101,            //    포트는 그대로
      "push_msg_ip":   "192.168.1.50",   // ★ 동일
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.50",   // ★ 동일
      "point_data_port": 56301,
      "imu_data_ip":   "192.168.1.50",   // ★ 동일
      "imu_data_port": 56401,
      "log_data_ip":   "",               //    빈 문자열 유지
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.147",             // ★ A-7에서 찾은 LiDAR IP
      "pcl_data_type": 1,                //    그대로
      "pattern_mode": 0,                 //    그대로
      "extrinsic_parameter": {           // ★★ 전부 0 유지 — 절대 채우지 말 것
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "x": 0, "y": 0, "z": 0
      }
    }
  ]
```

### 규칙 (틀리면 검사기가 잡습니다)

| 규칙 | 이유 |
|---|---|
| `host_net_info`의 4개 IP는 **전부 동일**하고, Computer 2에 **실제로 할당된 주소**여야 함 | SDK가 그 주소에 bind합니다. 없는 주소면 소켓이 아무것도 못 받고, **드라이버는 에러 없이 계속 돕니다** |
| LiDAR IP와 호스트 IP는 **같은 /24** | 아니면 라우팅을 명시해야 함 (경고) |
| `extrinsic_parameter`는 **전부 0** | 외부 파라미터(H-1)는 `g1_mid360.xacro`에만 존재합니다. 여기에도 넣으면 **두 번 적용**되어 모든 장애물이 어긋납니다 |
| 포트 번호는 손대지 않음 | 이미 사용 중이면 검사기가 알려줍니다 |

`livox_driver.yaml`은 **손대지 않습니다** (`xfer_format: 0`,
`frame_id: mid360_link`, `publish_freq: 10.0` — 검사기가 이 세 값을
확인합니다).

### 검사 (즉시)

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash && source install/setup.bash
python3 src/g1_perception/g1_perception_bringup/test/hw_config_check.py \
        src/g1_perception/g1_perception_bringup/config/MID360_config.json
echo "EXIT=$?"
```

기대 출력:
```
local IPv4 addresses: ['127.0.0.1', '192.168.1.50', '192.168.50.20']
hw_config_check: PASS
EXIT=0
```

| EXIT | 의미 |
|---|---|
| **0** | PASS — 다음 단계로 |
| **1** | FAIL — 출력된 `FAIL:` 줄을 고치십시오 |
| **2** | 아직 플레이스홀더 IP(`192.168.1.5` / `192.168.1.12`)가 남아 있음 |

### ★ 반드시 재빌드 — config는 "설치된 artefact"입니다

launch가 읽는 것은 소스 트리가 아니라 `install/` 안의 사본입니다.

```bash
cd ~/unitree_rl_mjlab/ros2
colcon build --packages-select g1_perception_bringup
source install/setup.bash
python3 src/g1_perception/g1_perception_bringup/scripts/config_diff.py
```

기대 출력:
```
config_diff: PASS   (installed launch/config/rviz match the source tree)
```

> 이 규칙은 YAML / launch / RViz 레이아웃을 **고칠 때마다** 적용됩니다.
> 재빌드하지 않으면 **예전 숫자로 실험하게 됩니다.**

---

## A-9. 네트워크 환경파일 `~/.g1_viz_env`

**실행 위치: 홈 디렉토리**

Computer 2 ↔ Computer 3 연결에 쓰는 값들입니다. 스크립트가 이 파일을 읽어
`RMW_IMPLEMENTATION` / `ROS_DOMAIN_ID` / `CYCLONEDDS_URI`를 설정합니다.

**NIC이 2개인 경우** (LiDAR 망과 노트북 망이 분리):

```bash
cat > ~/.g1_viz_env <<'EOF'
export G1_VIZ_DOMAIN_ID=7            # Computer 3과 반드시 동일한 값
export G1_VIZ_IFACE=wlan0            # ★ Computer 3(노트북)으로 나가는 NIC
export G1_SENSOR_IFACE=eth0          # ★ LiDAR/로봇 쪽 NIC (다를 때만)
#export G1_VIZ_PEER=192.168.50.30    # static 모드일 때: Computer 3의 IP
#export G1_SENSOR_PEER=192.168.1.147 # static + dual NIC일 때
EOF
```

**NIC이 1개인 경우** (전부 같은 망):

```bash
cat > ~/.g1_viz_env <<'EOF'
export G1_VIZ_DOMAIN_ID=7
export G1_VIZ_IFACE=wlan0            # 유일한 NIC 이름
#export G1_VIZ_PEER=192.168.50.30
EOF
```

`G1_SENSOR_IFACE`를 설정하면 dual-NIC용 XML이 **자동으로** 선택됩니다.

> **`ROS_DOMAIN_ID` 값 정하기.** 이 세션은 perception 전용이므로 **Computer
> 2와 3만 일치하면 됩니다.** 로봇 SDK2가 쓰는 도메인과 다른 값(예: 7)을
> 쓰면 discovery가 깨끗해집니다. 나중에 DPCBF control seam이 붙어
> unitree_sdk2와 같은 도메인이 필요해지면 그때 로봇 값으로 맞춥니다.

환경 적용 테스트:

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
source install/g1_perception_bringup/share/g1_perception_bringup/env/viz_env_computer2.sh multicast
```

기대 출력:
```
viz_env (computer2): mode=multicast
  ROS_DOMAIN_ID=7
  CYCLONEDDS_URI=file:///home/<user>/unitree_rl_mjlab/ros2/install/g1_perception_bringup/share/g1_perception_bringup/config/cyclonedds/c2_dual_nic_multicast.xml
  G1_VIZ_IFACE=wlan0  G1_SENSOR_IFACE=eth0
```

> **env 스크립트 경로 주의.** A-5에서 `--merge-install` **없이** 빌드했으므로
> 경로는
> `install/g1_perception_bringup/share/g1_perception_bringup/env/...` 입니다.
> (merge-install 레이아웃이면 `install/share/g1_perception_bringup/env/...`)
> 헷갈리면 찾아서 쓰십시오:
> ```bash
> find ~/unitree_rl_mjlab/ros2/install -name viz_env_computer2.sh
> ```

스크립트가 실패하면 실패 이유를 그대로 출력합니다 (`G1_VIZ_IFACE unset`,
`interface 'wlan0' does not exist on this machine` 등) — 그 줄을 고치면
됩니다.

---

## A-10. Preflight — 실행 전 마지막 관문

**실행 위치: `~/unitree_rl_mjlab/ros2`**

로봇 전원은 켜져 있고 **명령은 주지 않은** 상태에서 실행합니다. 이
스크립트는 노드를 하나도 띄우지 않고, LiDAR 소켓을 열지 않으며, 토픽을
publish하지 않습니다 — 전부 `ip`, `ping`, 파일 검사, 체크섬입니다.

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
source install/g1_perception_bringup/share/g1_perception_bringup/env/viz_env_computer2.sh multicast

# ★ 반드시 소스 트리 경로로 실행하십시오 (아래 박스 참조)
./src/g1_perception/g1_perception_bringup/scripts/g1_hw_preflight.sh
echo "EXIT=$?"
```

> ### ★ `ros2 run`이 아니라 소스 트리 경로로 실행하는 이유
>
> `ros2 run g1_perception_bringup g1_hw_preflight.sh`로 실행하면 **§4와 §7이
> 제대로 동작하지 않습니다**:
> - **§4 (source vs installed)**: `install-only machine: no source tree to
>   compare` — 소스와 설치본의 drift를 검사하지 못합니다.
> - **§7 (extrinsic guard)**: `t7_hw_extrinsic_guard.py`가 xacro를
>   `lib/g1_description/../urdf/`에서 찾는데 설치 레이아웃에는 그 경로가
>   없어 **오탐 FAIL**이 납니다.
>
> 소스 트리 경로로 실행하면 둘 다 정상 동작합니다 (Foxy 컨테이너에서
> 실측 확인 — 부록 1). Computer 2에는 소스 트리가 있으므로 그냥 이렇게
> 쓰십시오.

### 정상 출력 (전부 통과했을 때)

```
=== 1. ROS 2 environment
    ok   ROS_DISTRO=foxy
    WARN this workspace is pinned against humble, not foxy      <- 정상. 무시
    ok   a colcon prefix is on AMENT_PREFIX_PATH
    ok   RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ROS_DOMAIN_ID=7
    CYCLONEDDS_URI=file://...c2_dual_nic_multicast.xml
    cyclone interface: eth0
    ok   cyclone interface is not loopback

=== 2. required executables
    ok   livox_ros_driver2/livox_ros_driver2_node
    ok   direct_lidar_inertial_odometry/dlio_odom_node
    ok   g1_perception_utils/base_footprint_publisher
    ok   robot_state_publisher/robot_state_publisher
    ok   rclcpp_components/component_container
    WARN libpcl_ros_filters.so not found under the first AMENT prefix  <- 정상. 아래 참조

=== 3. installed package files
    ok   (13줄 전부 ok)

=== 4. source vs installed configuration
    ok   installed launch/config/rviz match the source tree

=== 5. Mid-360 network configuration
    ok   hw_config_check: PASS

=== 6. reachability
    ok   a non-loopback route to the LiDAR exists
    ok   LiDAR answers ICMP

=== 7. extrinsic guard (xacro <-> dlio.yaml)
    T7-hw: PASS - dlio.yaml extrinsics == xacro chain; frames disjoint
    ok   dlio.yaml extrinsics are derived from the xacro and agree with it

=== 8. storage for recording
    ok   at least 20 GB free

=== preflight summary
    hard failures : 0
    warnings      : 2

PREFLIGHT PASSED.
EXIT=0
```

### 정상적으로 뜨는 WARN 2개 (무시해도 됨)

| WARN | 이유 |
|---|---|
| `this workspace is pinned against humble, not foxy` | 스크립트가 Humble 기준으로 쓰였을 뿐. Foxy 지원은 패치 0011–0015로 완료되어 있고 게이트가 통과합니다 |
| `libpcl_ros_filters.so not found under the first AMENT prefix` | isolated 레이아웃이라 "첫 번째" prefix에 없을 뿐. A-6의 `find`가 파일을 찾았으면 실제로는 정상입니다 |

### EXIT 코드

| EXIT | 의미 | 조치 |
|---|---|---|
| **0** | 전부 통과 | 다음 단계 |
| **1** | HARD FAIL 있음 | **아무것도 launch하지 마십시오.** `FAIL` 줄을 고칠 것 |
| **2** | `MID360_config.json`이 아직 플레이스홀더 | A-8로 돌아가서 채우고 **재빌드** |

---

## A-11. 단계별 기동 검증 (3단계)

한 번에 전체 스택을 띄우지 말고 **아래 순서대로** 확인하십시오. 문제가
생겼을 때 어디가 원인인지 즉시 알 수 있습니다.

### 공통 env 블록 — **모든 새 터미널/pane에서 다시 붙여넣기**

> tmux의 새 pane은 tmux **서버가 시작된 시점의 환경**을 상속하므로
> `export`가 따라오지 않습니다. **"토픽 이름은 보이는데 데이터가 없다"의
> 최대 원인입니다.**

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
source install/g1_perception_bringup/share/g1_perception_bringup/env/viz_env_computer2.sh multicast
export SESSION=~/unitree_rl_mjlab/ros2/evidence/hardware/$(date +%F)/s1 && mkdir -p "$SESSION"
printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI
```

마지막 줄이 **세 값을 모두 출력**해야 합니다. 하나라도 비면 위 블록을 다시
실행하십시오.

---

### 단계 1 — LiDAR 드라이버만

**터미널 2개** (tmux pane 0, 1)

**pane 0** — 실행 위치 `~/unitree_rl_mjlab/ros2`
```bash
# (env 블록 먼저)
ros2 launch g1_perception_bringup source_hw.launch.py driver:=on lio:=off
```

**pane 1** — 실행 위치 아무 데나
```bash
# (env 블록 먼저)
ros2 topic list | grep livox
ros2 topic hz /livox/lidar        # Foxy: 토픽 1개씩만 됩니다
```
`Ctrl-C` 후:
```bash
ros2 topic hz /livox/imu
```

**정상 결과**

| 확인 | 기대값 |
|---|---|
| `ros2 topic list \| grep livox` | `/livox/lidar`, `/livox/imu` 두 줄 |
| `ros2 topic hz /livox/lidar` | **average rate: 10.0** 근처 |
| `ros2 topic hz /livox/imu` | **average rate: 200** 근처 |
| pane 0 드라이버 로그 | LiDAR 시리얼/IP를 찾았다는 로그, 에러 없음 |

점 개수와 프레임까지 확인:
```bash
ros2 topic echo /livox/lidar --once | head -20
```
`frame_id: mid360_link`, `width`가 수만 단위여야 합니다.

**아무것도 안 나오면** → LiDAR IP 또는 host IP가 틀렸습니다. A-7/A-8로
돌아가십시오. 드라이버는 **이 경우에도 에러 없이 계속 돕니다.**

pane 0 `Ctrl-C`.

---

### 단계 2 — 드라이버 + DLIO 오도메트리

> ⚠ **DLIO는 시작 시 로봇이 완전히 정지해 있어야 합니다.** IMU bias를
> `odom/imu/calibration/time: 3.0`초 동안 추정합니다. 이 3초 동안 로봇을
> 건드리지 마십시오.

**터미널 3개** (pane 0, 1, 2)

**pane 0** — 실행 위치 `~/unitree_rl_mjlab/ros2`
```bash
# (env 블록 먼저)
ros2 launch g1_perception_bringup source_hw.launch.py driver:=on lio:=dlio
```

**pane 1** — 실행 위치 아무 데나
```bash
# (env 블록 먼저)
ros2 topic hz /odom
```

**pane 2** — 정지 상태 drift 판정 (실행 위치 아무 데나)
```bash
# (env 블록 먼저)
ros2 run g1_perception_bringup hw_odom_drift.py --ros-args \
    -p duration:=60.0 -p json:=$SESSION/stage5_drift.json
```

**정상 결과**

| 확인 | 기대값 |
|---|---|
| `ros2 topic hz /odom` | **average rate: 100** 근처 |
| `hw_odom_drift.py` 진행 줄 | `drift 0.00x m (0.x cm/min) yaw 0.0x deg jump_max 0.00x m` |
| `hw_odom_drift.py` EXIT | **0** (기준: drift < 1 cm/min, 단일 점프 < 0.05 m) |

출력 예:
```
t+ 45.0 s   drift 0.004 m (0.5 cm/min)   yaw 0.06 deg   jump_max 0.001 m
            /odom 100.2 Hz   stamps ok
```

TF도 확인:
```bash
ros2 run tf2_ros tf2_echo odom base_link
```
translation이 거의 0이고 시간이 지나도 크게 변하지 않아야 합니다.

> **`/odom`이 존재한다고 오도메트리가 도는 것은 아닙니다.** DLIO는 시작
> 즉시 `/odom`을 publish하고 IMU가 오기 전까지 stamp를 0으로 찍습니다.
> `hw_odom_drift.py`는 stamp 0 샘플을 세어서 제외하고 그 사실을 알려줍니다 —
> "drift 0 cm/min"이 "완벽함"인지 "아무것도 안 돌고 있음"인지 구별해 줍니다.

pane 0 `Ctrl-C`.

---

### 단계 3 — 전체 perception 스택

**pane 0** — 실행 위치 `~/unitree_rl_mjlab/ros2`
```bash
# (env 블록 먼저)
ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
    driver:=on lio:=dlio \
    enable_plot_bridge:=true plot_publish_rate:=30.0
```

**정상 결과 — pane 1에서 하나씩** (Foxy는 `ros2 topic hz`에 토픽 1개씩):

```bash
ros2 topic hz /livox/lidar          # 10 Hz
ros2 topic hz /livox/imu            # 200 Hz
ros2 topic hz /odom                 # 100 Hz
ros2 topic hz /points_self_filtered # 10 Hz
ros2 topic hz /scan                 # 10 Hz
ros2 topic hz /raw_obstacles        # 10 Hz
ros2 topic hz /tracked_obstacles    # 10 Hz
ros2 topic hz /obstacles_safe       # 10 Hz
ros2 topic echo /diagnostics --once # 전 항목 level: 0 (OK)
```

**체인 의존성** — 위에서부터 순서대로 확인하십시오. `/scan`은 TF
`odom→base_footprint`가 있어야 나오고, 그 TF는 `/odom`이 있어야 나옵니다:

```
/livox/lidar ─> CropBox ─> /points_self_filtered ─> (TF 필요) ─> /scan
                                                        ↑
                                              /odom ─> base_footprint
```
따라서 **`/scan`이 비어 있으면 먼저 `/odom`을 의심**하십시오.

---

## A-12. Foxy 전용 주의사항 (반드시 읽을 것)

이 저장소의 다른 문서는 Humble 기준으로 쓰여 있습니다. Foxy에서 **실제로
다르게 동작하는 것**들입니다 (전부 실측 확인 — 부록 1).

### ① `record:=on` / `hw_record.sh`는 Foxy에서 **실패합니다**

두 경로 모두 `ros2 bag record --include-unpublished-topics`를 쓰는데, 이
옵션은 **Humble에서 추가된 것**입니다. Foxy에서는:

```
ros2: error: unrecognized arguments: --include-unpublished-topics
```

**Foxy용 대체 명령** (실행 위치: 아무 데나, env 블록 필요):

```bash
ros2 bag record -o "$SESSION/stage12_$(date +%H%M%S)" \
    /livox/lidar /livox/imu /odom /tf /tf_static \
    /points_self_filtered /scan /raw_obstacles \
    /tracked_obstacles /obstacles_safe /diagnostics
```

> ⚠ **스택이 완전히 올라온 뒤에 녹화를 시작하십시오.** 그 옵션의 존재
> 이유가 "아직 생성되지 않은 토픽도 녹화한다"였기 때문에, 옵션 없이는
> **녹화 시작 시점에 없는 토픽이 그 세션 내내 조용히 빠집니다.**
> A-11 단계 3의 rate 확인이 전부 끝난 뒤에 시작하십시오.
>
> `record:=on`은 launch 인자로 **쓰지 마십시오** (Foxy에서 그 프로세스만
> 죽고 스택은 계속 도는, 알아채기 어려운 상태가 됩니다).

### ② `ros2 topic hz`는 토픽을 **한 개만** 받습니다

```bash
ros2 topic hz /odom /obstacles_safe    # ✗ Foxy에서 에러
ros2 topic hz /odom                    # ✓
```

### ③ preflight의 humble WARN은 정상

`this workspace is pinned against humble, not foxy` — 스크립트의 문구일
뿐입니다. Foxy 지원은 패치 0011–0015로 완료되어 있습니다.

### ④ preflight는 소스 트리 경로로 실행

A-10의 박스 참조.

### ⑤ `ros2 pkg executables`도 패키지 1개씩

```bash
ros2 pkg executables livox_ros_driver2 direct_lidar_inertial_odometry  # ✗
```

### ⑥ Foxy ↔ Humble DDS interop — **검증 완료** (2026-08-07)

Foxy(deb rmw + 워크스페이스 CycloneDDS 0.10.2) ↔ Humble(소스 rmw +
CycloneDDS 0.10.2) 조합에서 이 프로젝트의 **4개 토픽 전부**가 **양방향으로
손실 없이** 전달되는 것을 실측했습니다. 커스텀 메시지(`Obstacles`,
`DpcbfPlotSample`)의 필드 값까지 검증했고, multicast·static peers 두 모드
모두 통과했습니다. 상세 수치는 [부록 1](#부록-1-검증-로그--이-문서의-어디까지가-실제로-확인된-것인가).

**단, 검증은 loopback(단일 호스트, 컨테이너 ↔ 호스트)에서 했습니다.**
즉 *distro를 건너는 것*(RMW · RTPS 와이어 · 타입 매칭 · QoS)은 증명됐고,
**실제 Wi-Fi/스위치 구간**(multicast 허용 여부, 방화벽, 대역폭)은 여전히
현장 변수입니다 — 그건 Foxy/Humble 문제가 아니라 네트워크 문제이며
Part C의 C-1/C-2가 그것을 판정합니다.

→ 실험 당일에도 **C-3의 2분 판정**은 그대로 하십시오. 이제는 "될까?"가
아니라 "이 망에서 되는가"를 확인하는 절차입니다.

---

# Part B — Computer 3 (노트북, ROS 2 Humble)

**Part B는 터미널 1개**로 전부 진행합니다.

## B-1. 사전 확인

**실행 위치: 홈 디렉토리**

```bash
cd ~
lsb_release -a                    # Ubuntu 22.04
source /opt/ros/humble/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"     # humble
ip -br addr                       # ★ Computer 2로 나가는 NIC 이름/IP 메모
```

## B-2. apt 의존성

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git \
    python3-colcon-common-extensions python3-vcstool \
    libarmadillo-dev libssl-dev libcunit1-dev \
    ros-humble-laser-geometry \
    python3-pyqtgraph python3-pyqt5 python3-pyqt5.qtopengl python3-matplotlib
```

- `libarmadillo-dev`, `ros-humble-laser-geometry` → `obstacle_detector` 빌드용
- `libssl-dev`, `libcunit1-dev` → `cyclonedds` 0.10.2 빌드용
- `python3-pyqtgraph` → GUI 기본 백엔드 (없으면 matplotlib으로 자동 fallback)

## B-3. clone + 외부 소스

**실행 위치: 홈 디렉토리 → `~/unitree_rl_mjlab/ros2`**

```bash
cd ~
git clone https://github.com/PSH0823/unitree_rl_mjlab.git
cd ~/unitree_rl_mjlab
git checkout obstacle_detection

cd ~/unitree_rl_mjlab/ros2
./setup_external.sh
```

## B-4. 빌드 — **11개 패키지**

**실행 위치: `~/unitree_rl_mjlab/ros2`**

Computer 3에는 perception도 DLIO도 livox도 필요 없습니다. 아래 11개가
**최소 집합**입니다 (`g1_perception_bringup`이 exec_depend로
`g1_perception_utils`/`sim_mjlidar_bridge`/`g1_description`을 요구하고,
그것들이 다시 `dpcbf_ros_adapter`/`sim_msgs`를 끌어옵니다 — 6개만 선택하면
colcon이 `install/share/<pkg>/package.sh`가 없다며 bringup에서 실패합니다.
2026-08-07 실측).

```bash
# B-2 목록에 없는 추가 의존성 (g1_perception_utils가 요구)
sudo apt-get install -y libyaml-cpp-dev ros-humble-diagnostic-msgs

cd ~/unitree_rl_mjlab/ros2

source /opt/ros/humble/setup.bash          # ★ 다른 워크스페이스는 source 금지

colcon build --merge-install --packages-select \
    cyclonedds rmw_cyclonedds_cpp obstacle_detector \
    dpcbf_viz_msgs dpcbf_plot_client \
    sim_msgs g1_description dpcbf_ros_adapter \
    sim_mjlidar_bridge g1_perception_utils \
    g1_perception_bringup
```

| 패키지 | 없으면 |
|---|---|
| `cyclonedds` + `rmw_cyclonedds_cpp` | **클라이언트가 기동조차 못 합니다.** 이 저장소는 CycloneDDS/rmw를 자체 빌드해 쓰고 `/opt/ros/humble`에는 없습니다 |
| `obstacle_detector` | 장애물 **원이 안 그려집니다** (나머지는 정상; 클라이언트 로그에 그렇게 찍힙니다) |
| `dpcbf_viz_msgs` | `/dpcbf/plot` 메시지 타입 |
| `dpcbf_plot_client` | 클라이언트 본체 |
| `g1_perception_bringup` | CycloneDDS XML + `viz_env_computer3.sh` |
| 나머지 5개 (`sim_msgs`, `g1_description`, `dpcbf_ros_adapter`, `sim_mjlidar_bridge`, `g1_perception_utils`) | bringup의 워크스페이스 의존성 — 없으면 bringup 빌드 자체가 실패합니다. 전부 경량입니다 |

> **빌드 전에 다른 워크스페이스를 source하지 마십시오.** colcon이 그것을
> underlay로 **체인**해서, 그 경로가 없는 환경에서 조용히 깨집니다.
> `/opt/ros/humble`만 source된 상태여야 합니다.
>
> `ros-humble-rmw-cyclonedds-cpp`를 apt로 깔아도 되지만(1.3.4), 그러면
> 저장소가 핀한 CycloneDDS 버전(0.10.2)과 달라지므로 위 방식을 권장합니다.

정상 종료:
```
Summary: 11 packages finished [x min]
```

검증:
```bash
cd ~/unitree_rl_mjlab/ros2
source install/setup.bash
ros2 pkg executables dpcbf_plot_client
find install -name "librmw_cyclonedds_cpp.so"
```
기대:
```
dpcbf_plot_client dpcbf_plot_client
dpcbf_plot_client synthetic_dpcbf_publisher
install/lib/librmw_cyclonedds_cpp.so
```

## B-5. 네트워크 환경파일

**실행 위치: 홈 디렉토리**

```bash
cat > ~/.g1_viz_env <<'EOF'
export G1_VIZ_DOMAIN_ID=7            # ★ Computer 2와 반드시 동일
export G1_VIZ_IFACE=wlp2s0           # ★ Computer 2로 나가는 이 노트북의 NIC
#export G1_VIZ_PEER=192.168.50.20    # static 모드일 때: Computer 2의 IP
EOF
```

적용 테스트:
```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/share/g1_perception_bringup/env/viz_env_computer3.sh multicast
```
기대:
```
viz_env (computer3): mode=multicast
  ROS_DOMAIN_ID=7
  CYCLONEDDS_URI=file://.../viz_multicast.xml
  G1_VIZ_IFACE=wlp2s0
```

> Computer 3은 `--merge-install`로 빌드했으므로 경로가
> `install/share/g1_perception_bringup/env/`입니다 (Computer 2와 다릅니다).

## B-6. 로봇 없이 GUI 미리 확인 (권장, 실험 전날)

**터미널 1개**, 실행 위치 `~/unitree_rl_mjlab/ros2`

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export G1_VIZ_ALLOW_LOCALHOST=1 G1_VIZ_DOMAIN_ID=42
source install/share/g1_perception_bringup/env/viz_env_computer3.sh localhost

ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py synthetic:=on
```

synthetic 소스가 "로봇이 원을 돌고 장애물 하나가 주기적으로 경로를
가로지르는" 시나리오를 만들어 줍니다. 화면 구성·조작·stale 표시를 여기서
미리 익혀 두십시오. `Ctrl-C`로 종료.

> ⚠ `synthetic:=on`은 `/odom`을 publish합니다. **실제 스택 옆에서 절대 켜지
> 마십시오** (`g1_perception_dpcbf.launch.py`는 `synthetic:=on stack:=hw`
> 조합을 아예 거부합니다).

---

# Part C — 두 컴퓨터 연결 (CycloneDDS)

## C-1. 먼저 판정: multicast가 되는가

**터미널 2개** (Computer 2에 1개, Computer 3에 1개)

**Computer 3** — 실행 위치 `~/unitree_rl_mjlab/ros2`
```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
source install/share/g1_perception_bringup/env/viz_env_computer3.sh multicast
ros2 multicast receive
```

**Computer 2** — 실행 위치 `~/unitree_rl_mjlab/ros2`
```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash && source install/setup.bash
source install/g1_perception_bringup/share/g1_perception_bringup/env/viz_env_computer2.sh multicast
ros2 multicast send
```

| 결과 | 다음 모드 |
|---|---|
| Computer 3에 `Received from ...: 'Hello World!'` | **multicast** 모드 사용 |
| 아무것도 안 옴 (타임아웃) | **static** 모드로 전환 (C-2) |

## C-2. static 모드 (multicast가 막힌 망)

랩 Wi-Fi / 관리형 스위치에서는 multicast가 막혀 있는 경우가 흔합니다.
그때는 상대방 IP를 명시합니다. **양쪽 모두** static이어야 합니다.

**Computer 2** — `~/.g1_viz_env`에 추가:
```bash
export G1_VIZ_PEER=192.168.50.30       # Computer 3의 IP
export G1_SENSOR_PEER=192.168.1.147    # G1_SENSOR_IFACE를 쓸 때만 필요
```
**Computer 3** — `~/.g1_viz_env`에 추가:
```bash
export G1_VIZ_PEER=192.168.50.20       # Computer 2의 IP
```
그리고 양쪽에서 `... viz_env_computer{2,3}.sh static`으로 source합니다.

## C-3. ★ Foxy ↔ Humble interop 판정 (2분)

> **distro 호환성 자체는 검증되었습니다** (2026-08-07, loopback 실측 —
> 4개 토픽 양방향 26/26 필드 검증, 손실 0, multicast·static 양쪽 모드.
> [부록 1](#부록-1-검증-로그--이-문서의-어디까지가-실제로-확인된-것인가)).
> 따라서 이 절차는 **"이 망에서 되는가"** 를 보는 것이지 "Foxy와 Humble이
> 원래 통하는가"를 보는 것이 아닙니다. 그래도 실험 시작 전에 반드시
> 하십시오 — 실패하면 원인은 도메인 ID / NIC / multicast / 방화벽입니다.

**터미널 2개**

**Computer 2** (env 블록 적용 후, 실행 위치 아무 데나):
```bash
ros2 topic pub /interop_probe std_msgs/msg/String "{data: 'hello from foxy'}" -r 2
```

**Computer 3** (env 적용 후, 실행 위치 아무 데나):
```bash
ros2 topic list | grep interop_probe
ros2 topic echo /interop_probe
```

| 결과 | 판정 |
|---|---|
| `data: hello from foxy`가 2 Hz로 찍힘 | ✅ **링크 정상.** 그대로 진행 |
| 토픽 이름만 보이고 데이터 0 | multicast 차단 → C-2의 static 모드로 |
| 토픽 이름도 안 보임 | `ROS_DOMAIN_ID` / `CYCLONEDDS_URI` / NIC 이름 / 방화벽 (Part E) |
| static으로도 안 됨 | 네트워크 문제입니다 (distro 문제가 아님 — §C-3 상단). 두 컴퓨터가 서로 `ping` 되는지부터 확인하고, 그래도 안 되면 C-4 fallback |

성공하면 실제 메시지 타입으로도 한 번 더 확인하십시오 (커스텀 메시지가
distro를 건너 매칭되는지):

```bash
# Computer 2에서 스택을 띄운 뒤, Computer 3에서
ros2 topic echo /obstacles_safe --once
```

## C-4. interop이 안 될 때의 fallback 3가지

| # | 방법 | 명령 |
|---|---|---|
| **1** | **Computer 2에서 GUI를 직접 띄우고 화면만 전달** — 가장 확실합니다. `dpcbf_plot_client`는 Foxy에서도 빌드되어 있습니다 (A-5의 18개에 포함) | Computer 3에서 `ssh -X <user>@<onboard>` 후, Computer 2에서 env 블록 + `ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py backend:=matplotlib`. A-2의 선택 apt 필요 |
| **2** | **콘솔 read-out으로 대체** — GUI 없이 SSH 콘솔에서 장애물 표를 봅니다 | Computer 2에서 `ros2 run g1_perception_bringup hw_obstacle_watch.py` |
| **3** | **bag 녹화 후 노트북에서 재생** — 실시간은 포기, 데이터는 확보 | Computer 2에서 A-12①의 녹화 명령 → `scp`로 노트북에 복사 → Computer 3에서 `ros2 bag play <bag>` + 플롯 클라이언트 |

> fallback 1이 실질적으로 가장 좋습니다. 화면만 X11로 넘기므로 DDS
> interop 문제와 완전히 무관하고, Wi-Fi 대역폭도 덜 씁니다.

---

# Part D — 실험 실행 시트 (복붙용)

로봇 정지 상태에서 물체의 실시간 상태 추정을 보는 **당일 전체 절차**입니다.

## D-0. 터미널 구성

| 컴퓨터 | 터미널 수 | 용도 |
|---|---|---|
| **Computer 2** | tmux 세션 1개 안에 **pane 4개** | 0=스택, 1=장애물 read-out, 2=녹화, 3=점검 |
| **Computer 3** | 일반 터미널 **2개** | 1=링크 확인, 2=플롯 GUI |
| | **합계 6개** | |

tmux pane 만들기 (Computer 2, SSH 접속 직후):
```bash
tmux new -s g1
# Ctrl-b "   → 가로 분할
# Ctrl-b %   → 세로 분할
# Ctrl-b 방향키 → pane 이동
# Ctrl-b q   → pane 번호 표시
```

## D-1. Computer 2 — pane별 복붙 블록

### 🔵 pane 0 — 스택 (메인)

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
source install/g1_perception_bringup/share/g1_perception_bringup/env/viz_env_computer2.sh multicast
export SESSION=~/unitree_rl_mjlab/ros2/evidence/hardware/$(date +%F)/s1 && mkdir -p "$SESSION"
printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI

# --- 1) preflight (PASS 아니면 여기서 멈출 것) ---
./src/g1_perception/g1_perception_bringup/scripts/g1_hw_preflight.sh | tee "$SESSION/preflight.txt"

# --- 2) 스택 기동 (로봇을 3초간 건드리지 말 것: DLIO IMU 캘리브레이션) ---
ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
    driver:=on lio:=dlio \
    enable_plot_bridge:=true plot_publish_rate:=30.0
```

### 🔵 pane 1 — 장애물 read-out (콘솔에서 보는 실시간 상태 추정)

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
source install/g1_perception_bringup/share/g1_perception_bringup/env/viz_env_computer2.sh multicast
export SESSION=~/unitree_rl_mjlab/ros2/evidence/hardware/$(date +%F)/s1

ros2 run g1_perception_bringup hw_obstacle_watch.py --ros-args \
    -p target_frame:=base_link -p rate:=2.0 \
    -p json:=$SESSION/obstacle_watch.jsonl
```

출력 예 (**이것이 "물체의 실시간 상태 추정"입니다**):
```
/tracked_obstacles  base_link   2 circles
  uid    x       y     range   bearing      r    true_r    |v|
    7   1.02   -0.03    1.02     -1.7deg  0.201   0.150   0.01
   11   2.45    0.88    2.60     19.7deg  0.370   0.200   0.62
```

| 컬럼 | 의미 |
|---|---|
| `uid` | 트랙 ID. 물체가 계속 보이는 동안 유지됩니다 |
| `x, y` | **로봇 기준(base_link)** 위치 [m] |
| `range, bearing` | 로봇으로부터의 거리 [m] / 방위 [deg] |
| `r` | 안전 마진이 더해진 반지름 |
| `true_r` | **측정된** 반지름 (정확도 판정은 이 값으로) |
| `\|v\|` | 속도 크기 [m/s] — **odom 기준 속도를 로봇 축으로 회전**한 것. 정지 물체는 로봇이 걸어도 ~0 |

### 🔵 pane 2 — bag 녹화 (**스택이 다 올라온 뒤에** 시작)

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
source install/g1_perception_bringup/share/g1_perception_bringup/env/viz_env_computer2.sh multicast
export SESSION=~/unitree_rl_mjlab/ros2/evidence/hardware/$(date +%F)/s1 && mkdir -p "$SESSION"

# ★ Foxy 전용 형태 (--include-unpublished-topics 없음, A-12① 참조)
ros2 bag record -o "$SESSION/stage12_$(date +%H%M%S)" \
    /livox/lidar /livox/imu /odom /tf /tf_static \
    /points_self_filtered /scan /raw_obstacles \
    /tracked_obstacles /obstacles_safe /diagnostics
```

### 🔵 pane 3 — 점검

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
source install/g1_perception_bringup/share/g1_perception_bringup/env/viz_env_computer2.sh multicast
export SESSION=~/unitree_rl_mjlab/ros2/evidence/hardware/$(date +%F)/s1

# 하나씩 확인 (Foxy는 토픽 1개씩). 각각 Ctrl-C 후 다음 줄로.
ros2 topic hz /livox/lidar          # 10
ros2 topic hz /livox/imu            # 200
ros2 topic hz /odom                 # 100
ros2 topic hz /scan                 # 10
ros2 topic hz /obstacles_safe       # 10
ros2 topic echo /diagnostics --once | tee "$SESSION/diagnostics.txt"
```

## D-2. Computer 3 — 터미널별 복붙 블록

### 🟢 터미널 1 — 링크 확인 (**GUI보다 먼저**)

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/share/g1_perception_bringup/env/viz_env_computer3.sh multicast
#   Computer 2와 같은 모드를 쓸 것 (multicast | static)

printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI

ros2 topic list | grep -E "odom|obstacles_safe|scan"
ros2 topic hz /odom                # 100 Hz 근처면 링크 정상
ros2 topic hz /obstacles_safe      # 10 Hz
```

**여기서 안 보이면 GUI를 띄워도 소용없습니다.** Part C / Part E로.

### 🟢 터미널 2 — 플롯 클라이언트 (GUI)

```bash
cd ~/unitree_rl_mjlab/ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/share/g1_perception_bringup/env/viz_env_computer3.sh multicast

ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py
```

옵션:
```bash
ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py \
    window_s:=60.0 gui_rate_hz:=20.0 stale_after_s:=1.0 backend:=matplotlib
```

| 인자 | 기본값 | 의미 |
|---|---|---|
| `backend` | `auto` | `pyqtgraph` \| `matplotlib`. auto는 pyqtgraph 우선 |
| `window_s` | `30.0` | 시계열 창 길이 [s] |
| `gui_rate_hz` | `25.0` | 화면 갱신률 |
| `stale_after_s` | `1.0` | 이 시간 넘게 안 오면 STALE 표시 |
| `obstacles_topic` | `/obstacles_safe` | 장애물 소스 |

## D-3. 화면 읽는 법

**좌측 (top-down 뷰)**
- 로봇 위치·heading·지나온 trail → `/odom`
- 장애물 **원** + 속도 방향 화살표 → `/obstacles_safe`
- 좌상단 배너: source별 상태

**배너 3줄 — 이것이 정상입니다:**

| 줄 | 정상 상태 |
|---|---|
| `odom` | 🟢 **`ok`** |
| `obstacles_safe` | 🟢 **`ok`** |
| `dpcbf/plot` | 🔴 **`NO DATA`** ← §0.3 참조. **이것이 정상입니다** |

**우측 (시계열 5단)** — 전부 `/dpcbf/plot` 기반이므로 이번 세션에서는
비어 있습니다. 정상입니다.

소스가 끊기면 해당 줄이 빨간 `STALE n.ns`로 바뀌고 **GUI는 계속
갱신됩니다** (멈춘 창은 아무것도 말해주지 않으므로 의도한 동작입니다).

## D-4. 실험 진행 — 물체를 어떻게 놓아야 보이는가

파이프라인의 실제 한계입니다. 이 범위를 벗어나면 **아무 에러 없이 그냥
안 보입니다.**

| 조건 | 값 | 출처 |
|---|---|---|
| **높이 대역** | 바닥 기준 **0.70 m ~ 1.50 m**만 사용 | `pointcloud_to_laserscan.yaml` `min_height`/`max_height` (frame: `base_footprint`) |
| **거리** | **0.3 m ~ 5.0 m** | 같은 파일 `range_min`/`range_max` |
| **크기** | 실제 반지름 **≲ 0.55 m** | `obstacle_detector.yaml` `max_circle_radius: 0.60` (fit 바이어스 +8.4% 포함) |
| **최소 점 개수** | 5점 이상 한 그룹 | `min_group_points: 5` |
| **로봇 자기 몸** | 센서 기준 ±0.40 m 박스 안은 제거 | `cropbox_self_filter.yaml` |
| **최소 출력 반지름** | 0.20 m 미만은 0.20으로 확대 | `safety_obstacle_filter.yaml` `min_radius` |

**따라서:**
- ✅ **사람** — 아주 잘 보입니다 (몸통이 0.7–1.5 m 대역에 있음)
- ✅ **키 1 m 이상의 상자·통·의자·삼각대**
- ❌ **바닥에 놓인 낮은 상자** (0.7 m 대역에 안 들어감)
- ❌ **넓은 벽·기둥·팔레트 더미** (원 모델 범위 밖 — 드롭되고 카운트됩니다)
- ❌ **5 m 밖 / 0.3 m 안**

**권장 시나리오 (로봇 정지):**

1. 로봇 앞 **2 m**에 사람이 서 있는다 → pane 1의 표에 `uid` 하나가 뜨고
   `x≈2.0, y≈0, |v|≈0`
2. 그 사람이 **좌우로 천천히 걷는다** → `y`가 변하고 `|v|`가 0.3–0.8 정도로
   올라감. GUI에서 원이 움직이고 속도 화살표가 생김
3. 사람이 **로봇 쪽으로 접근** → `range`가 줄고, GUI 원이 커집니다
4. 사람이 **5 m 밖으로 나감** → 트랙이 사라짐 (`tracking_duration: 1.0`초
   coast 후)
5. 물체 **2개** 배치 → `uid` 두 개가 각각 유지되는지 (association 확인)

각 단계마다 pane 2가 녹화 중인지, pane 3의 rate가 유지되는지 확인하십시오.

## D-5. 체크포인트 요약

| 위치 | 명령 | 정상값 |
|---|---|---|
| C2 pane 3 | `ros2 topic hz /livox/lidar` | 10 Hz |
| C2 pane 3 | `ros2 topic hz /livox/imu` | 200 Hz |
| C2 pane 3 | `ros2 topic hz /odom` | 100 Hz |
| C2 pane 3 | `ros2 topic hz /scan` | 10 Hz |
| C2 pane 3 | `ros2 topic hz /obstacles_safe` | 10 Hz |
| C2 pane 3 | `ros2 topic echo /diagnostics --once` | 전 항목 `level: 0` |
| C2 pane 1 | `hw_obstacle_watch.py` | 물체가 있을 때 표에 행이 생김 |
| C3 T1 | `ros2 topic hz /odom` | C2와 같은 값 |
| C3 T2 | GUI 배너 | `odom` ok / `obstacles_safe` ok / `dpcbf/plot` NO DATA |

기대 주기의 **절반 이하**로 떨어지면 `hw_diagnostics`가 ERROR로 잡습니다.

---

# Part E — 문제 해결

## E-1. Computer 2 쪽

| 증상 | 확인 순서 |
|---|---|
| `/livox/*` 토픽이 아예 없음 | ① `MID360_config.json`의 host IP가 **실제 로컬 IP**인가 (`hw_config_check.py`) ② LiDAR IP 재확인 (A-7) ③ `ping <lidar-ip>` ④ 케이블/전원 |
| `/livox/lidar`는 있는데 데이터 0 | LiDAR IP는 맞고 host IP가 틀린 경우가 대부분. `sudo tcpdump -ni eth0 udp port 56300`으로 점 데이터가 오는지 확인 |
| `/odom`이 안 나옴 | ① `/livox/imu`가 200 Hz인가 ② DLIO 로그에 에러 ③ 시작 시 로봇이 정지해 있었는가(IMU 캘리브 3초) |
| `/odom`은 나오는데 stamp가 0 | DLIO가 아직 IMU를 못 받음 → `/livox/imu` 확인 |
| `/scan`이 비어 있음 | ① `/odom` 먼저 (TF `odom→base_footprint`가 필요) ② 높이 대역 0.7–1.5 m 안에 물체가 있는가 ③ `ros2 run tf2_ros tf2_echo odom base_footprint` |
| `/raw_obstacles`는 있는데 비어 있음 | 물체가 D-4의 조건 밖. `ros2 topic echo /scan --once`로 유효 range 값이 있는지 |
| `/obstacles_safe`만 비어 있음 | `max_age: 0.30` 초과(입력이 느림) 또는 `max_circle_radius: 0.60` 초과로 드롭. safety filter 로그 확인 |
| CropBox 컴포넌트 로드 실패 | A-6의 `find install -name libpcl_ros_filters.so` 재확인 |
| preflight §7 FAIL | `ros2 run`으로 실행했을 것 → **소스 트리 경로**로 재실행 (A-10) |
| `record:=on`이 즉시 죽음 | Foxy에서는 정상 동작하지 않습니다 → A-12①의 대체 명령 |
| launch는 뜨는데 예전 값으로 도는 듯 | config 재빌드 안 함 → `colcon build --packages-select g1_perception_bringup` → `config_diff.py` PASS |

## E-2. Computer 2 ↔ Computer 3 링크

| 증상 | 확인 순서 |
|---|---|
| Computer 3의 `ros2 topic list`가 빔 | ① **양쪽** `printenv ROS_DOMAIN_ID CYCLONEDDS_URI RMW_IMPLEMENTATION`이 일치하는가 ② tmux pane마다 env를 다시 적용했는가 ③ 두 컴퓨터가 서로 `ping` 되는가 |
| 토픽 **이름은 보이는데 데이터 0** | multicast 차단 → **양쪽** static 모드 (C-2). `ros2 multicast send`/`receive`로 판정 |
| Computer 2에서만 안 나감 | 센서망/시각화망이 다른 NIC → `G1_SENSOR_IFACE` 설정 (dual-NIC XML 자동 선택). **`CYCLONEDDS_URI`는 프로세스당 하나이며 기존 `~/cyclonedds.xml`과 병합되지 않습니다** |
| 방화벽 의심 | UDP **7400–7500** (도메인 ID에 따라 이동). `sudo ufw status` |
| `/odom`은 오는데 `/obstacles_safe`만 안 옴 | Computer 3에 `obstacle_detector`가 안 깔림 → B-4 재확인. 클라이언트 로그에 `obstacle_detector msgs not installed`로 찍힙니다 |
| 클라이언트가 기동하다 즉사 | `librmw_cyclonedds_cpp.so` 없음 → B-4의 6개 패키지 재확인 |
| 다 맞는데 안 됨 | **Foxy↔Humble interop 실패 가능성** → C-3 판정 → C-4 fallback |
| Wi-Fi가 느림 | `/odom`+`/obstacles_safe`는 raw cloud 없이 수백 kB/s 미만이어야 정상. raw PointCloud2는 절대 넘기지 마십시오 |

## E-3. GUI

| 증상 | 확인 |
|---|---|
| 창이 안 뜸 | `backend:=matplotlib`로 재시도. `python3 -c "import pyqtgraph"` |
| 원은 그려지는데 로봇이 안 움직임 | 로봇이 정지 상태면 정상입니다 |
| 우측 그래프가 전부 빔 | **정상** — `/dpcbf/plot` publisher가 없습니다 (§0.3) |
| 배너가 `STALE`로 바뀜 | 해당 소스가 끊긴 것. Computer 2의 rate 확인 |

---

# Part F — 세션 종료와 기록

## F-1. 종료 순서

```bash
# ① Computer 3 — 클라이언트 Ctrl-C (Computer 2에 아무 영향 없음)
# ② Computer 2 pane 2 — bag Ctrl-C (rosbag이 마무리될 때까지 기다릴 것)
# ③ Computer 2 pane 1 — read-out Ctrl-C
# ④ Computer 2 pane 0 — 스택 Ctrl-C
```

> 일부 경로에서 **livox 드라이버가 SIGINT를 무시**합니다. `Ctrl-C` 후에도
> 남아 있으면:
> ```bash
> pgrep -af livox_ros_driver2_node
> pkill -f livox_ros_driver2_node
> ```

## F-2. 기록 마무리 — **로봇이 아직 눈앞에 있을 때**

```bash
cd ~/unitree_rl_mjlab/ros2
du -sh "$SESSION"          # bag 크기 기록
ls -la "$SESSION"
```

사람만 아는 메타데이터를 채웁니다 (기계가 아는 것 — commit, 환경, 로드된
YAML 체크섬, 네트워크 — 은 스크립트가 자동으로 넣습니다):

```bash
BAG=$SESSION/stage12_<시각>          # 실제 bag 경로로 바꿀 것
ros2 run g1_perception_bringup hw_session_metadata.py \
    --out "$BAG.session.json" --bag "$BAG" --copy-configs \
    --g1-variant "G1 EDU" --lidar-serial <시리얼> --operator "<이름>" \
    --robot-state Passive --scenario "<물체 배치 설명>"
echo "EXIT=$?"
#   EXIT 0 = 완료.  EXIT 1 이면 출력된 빈 필드를 채워 다시 실행
```

## F-3. Computer 3으로 회수

**Computer 3에서 실행:**
```bash
mkdir -p ~/g1_sessions
scp -r <user>@<onboard-pc>:~/unitree_rl_mjlab/ros2/evidence/hardware/$(date +%F) \
       ~/g1_sessions/
```

> **분석은 현장에서 하지 않습니다.** provenance를 나중에 재구성한 bag은
> evidence가 아닙니다. 현장에서는 capture만 하고, 판단 근거가 되는 출력은
> 전부 `$SESSION`에 파일로 남기십시오.

## F-4. 절대 규칙 (요약)

1. **bag 없는 stage는 실패한 stage입니다.** 로봇 시간에 debug하지 말고
   capture하십시오.
2. **tmux 밖에서 launch/record 금지.** SSH 끊김 = bag 잘림.
3. **모든 pane에서 env 재적용.** 특히 `ROS_DOMAIN_ID`, `CYCLONEDDS_URI`.
4. **`use_sim_time`은 hardware에서 어디서나 false**이며 launch 인자도
   아닙니다 (`/clock`이 없으므로 true인 노드는 타이머가 영영 안 돕니다).
5. **config는 설치된 artefact입니다.** YAML/launch/RViz 수정 후에는
   `colcon build --packages-select g1_perception_bringup` →
   `config_diff.py` PASS여야 적용됩니다.
6. **Computer 3은 표시 전용.** 죽어도 Computer 2에 영향이 없습니다.
7. **이 세션에 actuation은 없습니다.**

---

# 부록 1. 검증 로그 — 이 문서의 어디까지가 실제로 확인된 것인가

2026-08-06 / 08-07, 개발 머신 + `g1-perception:foxy` focal 컨테이너에서 실측.

## ✅ Foxy ↔ Humble DDS interop — 실측 결과 (2026-08-07)

**구성**: `g1-perception:foxy` 컨테이너를 `--network host`로 띄워
호스트의 Humble 워크스페이스와 같은 네트워크 네임스페이스에 둠. 양쪽 모두
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, 같은 `ROS_DOMAIN_ID`, 저장소의
CycloneDDS XML 사용. **로봇에서 쓸 조합과 동일**함을 먼저 확인:

```
Foxy 쪽 : /opt/ros/foxy/lib/librmw_cyclonedds_cpp.so   (deb, 2023 빌드)
           └─ libddsc.so.0 => install_foxy/unitree_sdk2/lib/libddsc.so.0   ← 워크스페이스 0.10.2
Humble 쪽: install/lib/librmw_cyclonedds_cpp.so         (소스 빌드)
           └─ libddsc.so.0 => install/lib/libddsc.so.0                     ← 워크스페이스 0.10.2
```

즉 `README.md`가 서술한 R-3 배치(0.7 빌드 rmw가 0.10.2 libddsc를 로드)
그대로입니다.

**테스트한 토픽 = Computer 3이 실제로 구독하는 4개**, 각각 실제 producer와
동일한 QoS로 publish:

| 토픽 | 타입 | publisher QoS | 목표 |
|---|---|---|---|
| `/interop_probe` | `std_msgs/String` | Reliable KeepLast(10) | 2 Hz |
| `/odom` | `nav_msgs/Odometry` | Reliable KeepLast(10) (DLIO, 패치 0006) | 100 Hz |
| `/obstacles_safe` | `obstacle_detector/Obstacles` | Reliable KeepLast(1) (safety filter) | 10 Hz |
| `/dpcbf/plot` | `dpcbf_viz_msgs/DpcbfPlotSample` | BestEffort KeepLast(5) (viz publisher) | 30 Hz |

구독 측은 `data_hub.py`와 동일하게 **전부 BestEffort**.

### 결과

| # | 방향 / 모드 | 수신 (15 s 창) | 필드 검증 |
|---|---|---|---|
| 1 | **Foxy → Humble**, multicast(`localhost.xml`) | probe 30 @2.00 Hz / odom **1500 @99.99 Hz** / obstacles 150 @10.00 Hz / plot 450 @30.00 Hz | **26/26 PASS** |
| 2 | **Humble → Foxy**, multicast | probe 24 @2.00 / odom 1199 @99.99 / obstacles 120 @10.00 / plot 360 @30.00 | **26/26 PASS** |
| 3 | **Foxy → Humble**, **static peers**(`viz_static_peers.xml`, `AllowMulticast=false`) | probe 24 @2.00 / odom **1200 @100.00 Hz** / obstacles 120 @10.00 / plot 360 @30.00 | **26/26 PASS** |

**패킷 손실 0** (12 s × 100 Hz = 1200 기대 → 1200 수신).

### 필드 단위로 확인된 것 (단순 도착이 아니라 payload)

- `std_msgs/String` 문자열, `nav_msgs/Odometry`의 `frame_id`/`child_frame_id`/
  pose(1e-9 정밀도)/`header.stamp`
- `obstacle_detector/Obstacles`: 중첩 배열 `CircleObstacle[]`, `uid`(uint64),
  `geometry_msgs/Point`·`Vector3`, `string semclass`,
  **`float64[3] covariance` (패치 0007로 추가한 필드)**
- `dpcbf_viz_msgs/DpcbfPlotSample`: `uint64 tick`, uint8 상수(`MODE_ESTIMATED`),
  중첩 메시지 `VelocityCommand2D` ×3, bool ×3, int32 ×3, 음수 float(`min_h`),
  `float64[3] acceleration`, **중첩 가변 배열 `PlotObstacle[]`**, `uint32`

### 프로덕션 클라이언트 코드로도 확인

`dpcbf_plot_client.data_hub`의 **`HubRunner`+`DataHub`**(플롯 클라이언트가
실제로 생성하는 그 객체)를 Humble에서 띄워 Foxy publisher에 붙임:

```
obstacles_available (obstacle_detector msgs built): True
  banner: plot=ok(0.030s,n=30)   odom=ok(0.003s,n=100)   obstacles=ok(0.063s,n=10)
  ...
  banner: plot=ok(0.012s,n=361)  odom=ok(0.009s,n=1201)  obstacles=ok(0.077s,n=120)

odom          : {'x': -0.2719, 'y': -0.9623, 'yaw': 0.0, 'vx': 0.2, ...}
trail points  : 1201
obstacles     : [{'id': 7, 'x': 1.0, 'radius': 0.251, 'true_radius': 0.2, 'vx': 0.31, ...},
                 {'id': 8, 'x': 2.0, 'radius': 0.351, 'true_radius': 0.3,  'vx': 0.62, ...}]
sample.tick   : 664      sample.min_h : -0.02      sample.safe : (0.3, 0.05, 0.1)
series safe_sagittal n=361   min_h n=361   intervention n=361   latency n=361 (≈1 ms)
RESULT: PASS - production client renders live Foxy data
```

즉 **GUI 배너 세 줄이 전부 녹색 `ok`**, 장애물 원 2개가 좌표·반지름·속도까지
정확히 파싱되고, 시계열 버퍼가 정상적으로 채워집니다.

### 이 검증의 한계 (남는 현장 변수)

loopback(단일 호스트)에서 했으므로 **distro를 건너는 층**(RMW · RTPS 와이어 ·
타입 매칭 · QoS 협상 · 메시지 직렬화)만 증명됐습니다. **실제 두 대 사이의
물리 네트워크**(Wi-Fi multicast 허용 여부, 방화벽 UDP 7400–7500, 대역폭,
NIC 선택)는 여전히 현장에서 C-1/C-3로 판정해야 합니다 — 그러나 그것이
실패하면 원인은 **네트워크이지 Foxy/Humble 조합이 아닙니다.**

재현 스크립트: `interop_pub_foxy.py` / `interop_sub_humble.py` /
`real_client_check.py` (세션 scratchpad. 저장소에 커밋되어 있지 않음)

---

## ✅ 그 밖에 이 문서를 쓰면서 직접 확인한 것

| 항목 | 결과 |
|---|---|
| `hw_offline_gates` (Foxy, 현재 트리) | **283/283 PASS** |
| `g1_perception_dpcbf.launch.py`가 Foxy launch에서 construct | ✅ 20 entities |
| `g1_perception_hardware_only` / `source_hw` / `description` / `perception` / `record_hw` construct | ✅ 전부 |
| Foxy `ros2 bag record --include-unpublished-topics` | ❌ **`unrecognized arguments`** → A-12① |
| Foxy `ros2 topic hz`가 받는 토픽 수 | **1개만** (Humble도 1개) |
| Foxy `ros2 pkg executables`가 받는 패키지 수 | **1개만** |
| Foxy `ros2 multicast` verb | ✅ 존재 |
| `g1_hw_preflight.sh`를 `ros2 run`으로 실행 | §4 = `install-only machine`, §7 = **오탐 FAIL** |
| `g1_hw_preflight.sh`를 **소스 트리 경로**로 실행 | §4 `ok`, §7 `T7-hw: PASS` → A-10 |
| `hw_config_check.py`를 실제 로컬 IP로 채운 JSON에 실행 | **`hw_config_check: PASS`, EXIT 0** |
| `install_foxy`에 19개 패키지 전부 존재 | ✅ (당시 레이아웃. 2026-08-07부터 워크스페이스는 18개 + underlay `cyclonedds` — A-5) |
| `g1_perception_bringup` 설치 실행파일 13개 | ✅ |
| `xacro`가 Foxy에서 `g1_mid360.xacro`를 처리 | ✅ (실패는 preflight의 경로 문제였음) |
| `viz_env_computer{2,3}.sh`가 isolated/merged 양쪽 레이아웃에서 XML을 찾음 | ✅ |
| Livox SDK2의 LiDAR 탐색 경로 (`255.255.255.255:56000`, 1초 주기) | ✅ 소스에서 확인 → A-7 방법 3 |

## ⚠ 확인되지 않은 것 (실기 당일 확인 필요)

| 항목 | 대응 |
|---|---|
| 실제 두 대 사이의 **물리 네트워크** (Wi-Fi multicast, 방화벽, 대역폭) — distro 호환성은 위에서 검증됨 | C-1 multicast 판정 → C-3 2분 판정 → 안 되면 C-2 static → C-4 fallback |
| 실제 Mid-360 하드웨어에서의 데이터 (rate, 점 개수, 노이즈) | A-11 단계 1 |
| 실제 로봇에서의 DLIO 오도메트리 품질 | A-11 단계 2 (`hw_odom_drift.py`) |
| Mid-360 내부 lidar→IMU 오프셋이 이 유닛과 맞는지 | `dlio.yaml` 헤더 참조. 5 cm 레버암 — 정지 시 무관 |
| 실제 self-hit 패턴에 맞는 CropBox 값 | 현재 값은 시뮬레이션에서 튜닝된 것 |
| `measurement_variance: 1.0` | **알려진 잘못된 값**입니다 (1 m 1σ 측정을 주장). 하드웨어 데이터로 재도출 필요 — `obstacle_detector.yaml` 주석 참조 |
| 온보드 PC에서의 CPU 부하 | `ros2 run g1_perception_bringup ...` 대신 `top`으로 관찰 |
| Foxy에서 `--merge-install` 빌드 | 미검증. A-5는 검증된 isolated 레이아웃을 씁니다 |

---

# 부록 2. 전체 파이프라인 데이터 흐름

```
[Mid-360]
   │ UDP 56300 (points) / 56400 (IMU) → host_net_info의 IP:포트
   ▼
livox_ros_driver2_node                                   ← MID360_config.json + livox_driver.yaml
   ├─ /livox/lidar   PointCloud2  10 Hz  frame=mid360_link
   └─ /livox/imu     Imu         200 Hz  frame=mid360_link
   │
   ├──────────────────────────────────────┐
   ▼                                      ▼
dlio_odom_node                      perception_container (단일 프로세스, intra-process)
   │  ← dlio.yaml                     │
   │    (extrinsics는 g1_mid360.xacro   ├─ pcl_ros::CropBox            ← cropbox_self_filter.yaml
   │     에서 derive됨)                 │     /livox/lidar → /points_self_filtered
   ├─ /odom  Odometry 100 Hz           │     (로봇 자기 몸 ±0.40 m 제거)
   └─ TF odom→base_link                │
        │                              ├─ pointcloud_to_laserscan     ← pointcloud_to_laserscan.yaml
        ▼                              │     /points_self_filtered → /scan
robot_state_publisher                  │     (높이 0.70–1.50 m 대역만, 거리 0.3–5.0 m)
   │  ← g1_mid360.xacro                │     target_frame=base_footprint  ← TF 필요!
   └─ TF base_link→torso_link→mid360_link
        │                              ├─ obstacle_extractor           ← obstacle_detector.yaml
base_footprint_publisher               │     /scan → /raw_obstacles  (원 fitting, odom frame)
   └─ TF odom→base_footprint           │
                                       ├─ obstacle_tracker            ← obstacle_detector.yaml
                                       │     /raw_obstacles → /tracked_obstacles
                                       │     (Kalman, 측정 도착 시 predict+correct,
                                       │      위치·속도·반지름 + 공분산)
                                       │
                                       └─ safety_obstacle_filter      ← safety_obstacle_filter.yaml
                                             /tracked_obstacles → /obstacles_safe
                                             (max_age 0.30 s, +0.051 m 고정 팽창,
                                              0.12 s 지연 보상 외삽, min_radius 0.20)
                                       │
hw_diagnostics ─ /diagnostics 10 Hz    │
                                       ▼
                        ═══════ CycloneDDS / Wi-Fi ═══════
                                       ▼
                        [Computer 3] dpcbf_plot_client
                          구독: /odom, /obstacles_safe, /dpcbf/plot(없음)
                          전부 BestEffort → 역압 없음, 죽어도 무해
```

---

## 관련 문서

| 상황 | 문서 |
|---|---|
| 3대 구성(Blackbox 포함) 당일 운용 | [`README.md`](README.md) |
| stage별(0–14) 정밀 검증 절차 | [`g1_first_perception_experiment.md`](g1_first_perception_experiment.md) |
| 로봇 전원 켜기 전에 알아야 할 것 | [`g1_hardware_preflight.md`](g1_hardware_preflight.md) |
| 검증된 것 / 안 된 것 구분 | [`g1_hardware_code_audit.md`](g1_hardware_code_audit.md) |
| plot 링크 상세 (토픽/QoS/CycloneDDS) | [`dpcbf_plot_visualization.md`](dpcbf_plot_visualization.md) |
| 스택 실행 일반·시뮬레이션 | [`operator_runbook.md`](operator_runbook.md) |
| 파이프라인 내부 동작·수치 근거 | [`pipeline_technical_report.md`](pipeline_technical_report.md) |
| 워크스페이스 구성·pin·patch·Foxy 이식 | [`../README.md`](../README.md) |
