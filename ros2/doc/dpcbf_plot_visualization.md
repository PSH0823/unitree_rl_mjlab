# DPCBF 실시간 plotting — Computer 2 → Computer 3 시각화 링크

Computer 2(G1 onboard)에서 돌아가는 perception + DPCBF의 상태를 Computer 3
(로컬 노트북)에서 실시간 plot으로 보기 위한 경로의 실행/검증 문서입니다.

```
Computer 2 (onboard)                          Computer 3 (laptop)
────────────────────────                      ─────────────────────
perception stack ──→ /odom ──────────────┐
   (기존 그대로)      /obstacles_safe ────┤    CycloneDDS
                                          ├──────────────→ dpcbf_plot_client
DPCBF control seam                        │                (pyqtgraph GUI)
  1 kHz Filter() ─ Push() ─→ 30 Hz        │
  DpcbfVizPublisher ─→ /dpcbf/plot ───────┘
```

원칙:

* **기존 토픽은 재사용한다.** `/odom`, `/obstacles_safe`는 Computer 3이 직접
  subscribe한다. 중복 publish 없음.
* **DPCBF 내부에만 있던 값만 새 토픽으로 낸다.** nominal/safe command,
  intervention, barrier h, QP 결과, staleness — 전부 `/dpcbf/plot` 한 개
  (`dpcbf_viz_msgs/DpcbfPlotSample`, 기본 30 Hz).
* **control loop는 절대 블록되지 않는다.** 1 kHz seam은 `Push()`(try_lock 뒤
  고정 크기 memcpy)만 호출하고, serialization/DDS는 전용 스레드가 한다.
  클라이언트/네트워크가 죽어도 control 쪽은 아무 것도 느끼지 못한다
  (BestEffort + 자체 타이머 스레드; 아래 검증 참조).
* **Computer 3은 read-only.** dpcbf_plot_client는 subscribe만 하며 robot이
  소비하는 어떤 토픽도 publish하지 않는다.
* raw PointCloud2는 시각화 목적으로 전송하지 않는다.

---

## 1. 구성 요소

| 위치 | 역할 |
|---|---|
| `ros2/src/g1_perception/dpcbf_viz_msgs/` | `DpcbfPlotSample` / `PlotObstacle` / `VelocityCommand2D` 메시지. Computer 3 최소 설치 대상 |
| `dpcbf_ros_adapter/include/.../viz_publisher.h`, `src/viz_publisher.cpp` | `DpcbfVizPublisher`: control→plot seam (mailbox + 30 Hz 타이머 노드) |
| `dpcbf_ros_adapter/include/.../adapter_config.h` | `LoadVizBridgeConfig()`: yaml `plot_bridge:` 섹션 로더 |
| `g1_perception_bringup/config/dpcbf_ros_adapter.yaml` | `plot_bridge:` 섹션 = plot bridge의 control surface |
| `simulate/src/main.cc` | ROS2 빌드의 axis_filter seam에서 매 tick `Push()` |
| `ros2/src/g1_perception/dpcbf_plot_client/` | Computer 3 클라이언트 (pyqtgraph, matplotlib fallback) + synthetic source |
| `g1_perception_bringup/launch/g1_perception_dpcbf.launch.py` | Computer 2 진입점 (hardware stack 포함 + plot env) |
| `dpcbf_plot_client/launch/dpcbf_plot_client.launch.py` | Computer 3 진입점 |
| `g1_perception_bringup/config/cyclonedds/*.xml` | CycloneDDS 설정 (multicast / static peer / dual-NIC / localhost) |
| `g1_perception_bringup/scripts/viz_env_computer{2,3}.sh`, `viz_env.example` | 환경 설정 (sourced) |

## 2. 토픽과 QoS

| 토픽 | 타입 | 주기 | QoS (publisher) | 비고 |
|---|---|---|---|---|
| `/dpcbf/plot` | `dpcbf_viz_msgs/DpcbfPlotSample` | 20–50 Hz (기본 30) | BestEffort KeepLast(5) | 신규. control tick 하나의 일관된 스냅샷 |
| `/odom` | `nav_msgs/Odometry` | DLIO 주기 | Reliable | 기존 그대로 |
| `/obstacles_safe` | `obstacle_detector/Obstacles` | ~10 Hz | Reliable KeepLast(1) | 기존 그대로 |
| `/dpcbf/status` | `diagnostic_msgs/DiagnosticArray` | 10 Hz | Reliable | 기존 그대로 (참고용) |

클라이언트의 subscription은 **전부 BestEffort**다. BestEffort request는
Reliable/BestEffort offer 양쪽과 호환되므로 QoS 불일치가 생길 수 없고,
클라이언트가 publisher에게 재전송 부하를 되돌려줄 방법도 없다.

`DpcbfPlotSample` 내용: tick, t_ctrl, mode(oracle/shadow/estimated), robot
(x,y,φ,v_sag,v_lat), **nominal/scaled/safe command**, command_scale, QP 결과
(solved, active·dpcbf·ecbf constraints, acceleration), **intervention**
(safe≠scaled), **min_h / min_clearance**, staleness(state, age), 그리고
필터가 이번 tick에 실제로 소비한 obstacle 목록(≤10개; id, x, y, r, v, 거리,
per-obstacle h). 전체 obstacle 스트림이 필요하면 `/obstacles_safe`를 그대로
보면 된다.

**시간축 규칙**: 클라이언트의 모든 시계열은 **수신 시각(로컬 monotonic)**
기준이므로 두 컴퓨터의 clock 동기화가 필요 없다. `header.stamp` 기반
latency 곡선은 "plot latency (needs NTP)"로 표시되며 NTP/chrony 없이는
참고용이다. stale 판정도 수신 시각 기준이라 clock skew와 무관하다.

## 3. plot bridge 설정 (Computer 2)

`config/dpcbf_ros_adapter.yaml`:

```yaml
plot_bridge:
  enabled: true
  topic: /dpcbf/plot
  rate_hz: 30.0        # 설계 대역 20–50, 로더가 [1,100] 강제
  frame_id: odom
```

env 오버라이드(재빌드 없이 1회성): `UNITREE_DPCBF_PLOT=0|1`,
`UNITREE_DPCBF_PLOT_RATE=<hz>`. 섹션/키 누락은 시작 시 loud fail — 켜져
있다고 믿는 시각화가 조용히 꺼져 있는 상태를 만들지 않기 위함이다.

1 kHz(또는 500 Hz A-mode) 계산 주기와 무관하게 토픽에는 최대 rate_hz
샘플/초만 나간다. 같은 tick은 두 번 publish되지 않으므로 control loop가
멈추면 토픽도 멈추고, 클라이언트에는 STALE로 정확히 표시된다.

## 4. 실기 실험 명령어 시트 (LiDAR + DLIO 세팅 완료 가정)

> ### ⚠ 먼저 알아야 할 것: 지금 hardware에는 DPCBF control seam이 없다
>
> `DpcbfVizPublisher`와 `ObstacleSource`를 생성하는 곳은 저장소 전체에서
> `simulate/src/main.cc`(MuJoCo 시뮬레이터) **하나뿐**이다. 로봇에서 도는
> `g1_ctrl`(deploy/)에는 DPCBF 참조가 0건이며,
> [`g1_first_perception_experiment.md`](g1_first_perception_experiment.md) §9가
> 명시하듯 hardware `RobotState` source와 command seam은 아직 **설계 노트**다.
>
> 따라서 **실기 세션에서 `/dpcbf/plot`은 publisher가 없다.** 클라이언트는
> 그 소스를 빨간 `NO DATA`로 정확히 표시하고, 나머지는 정상 동작한다.
> 실기에서 실제로 살아 있는 것:
>
> | 화면 요소 | 실기에서 | 출처 |
> |---|---|---|
> | 로봇 위치·heading·trail | ✅ 나옴 | `/odom` (DLIO) |
> | 장애물 원 + 속도 벡터 | ✅ 나옴 | `/obstacles_safe` |
> | source별 stale 표시 | ✅ 나옴 | 수신 시각 |
> | nominal/safe command 화살표·시계열 | ❌ 빈 화면 | `/dpcbf/plot` (미구현) |
> | intervention · min_h · command_scale | ❌ 빈 화면 | 〃 |
>
> 즉 이번 실기에서 이 클라이언트는 **"노트북에서 보는 실시간 perception
> 뷰어"**(지금까지 SSH 콘솔의 `hw_obstacle_watch.py`가 하던 일의 그래픽
> 버전)로 쓰인다. §9 hardware seam이 붙는 순간 Computer 3은 **아무 변경 없이**
> 나머지 절반이 채워진다 — 클라이언트는 이미 그 토픽을 구독하고 있다.

### 4.0 사전 준비 (실험 전날, 한 번만)

**Computer 2 (G1 onboard)** — 워크스페이스는 이미 빌드되어 있다고 가정.
`~/.g1_viz_env`만 만든다 (`viz_env.example` 복사):

```bash
cat > ~/.g1_viz_env <<'EOF'
export G1_VIZ_DOMAIN_ID=7          # Computer 1/2/3 공통. 로봇 값에 맞출 것
export G1_VIZ_IFACE=wlan0          # Computer 3(노트북)로 나가는 NIC
#export G1_SENSOR_IFACE=eth0       # Computer 1(LiDAR망)이 다른 NIC일 때만
#export G1_VIZ_PEER=192.168.50.30  # static 모드일 때 Computer 3 IP
#export G1_SENSOR_PEER=192.168.123.120
EOF
ip -br addr        # NIC 이름과 IP를 여기서 확인해 위에 적는다
```

**Computer 3 (노트북)** — ROS 2 Humble만 설치된 상태에서 시작.
`dpcbf_viz_msgs`·`dpcbf_plot_client`만으로는 **부족하다**: 이 저장소는
CycloneDDS/rmw를 자체 빌드해 쓰므로 rmw가 없으면 클라이언트가 기동조차
못 하고, 장애물 layer에는 `obstacle_detector` 메시지가 필요하다.
아래 6개가 검증된 최소 집합이다 (perception·DLIO·livox·DPCBF 없음):

```bash
sudo apt install -y libarmadillo-dev ros-humble-laser-geometry \
                    python3-pyqtgraph python3-pyqt5     # pyqtgraph 없으면 matplotlib fallback
git clone <이 저장소> ~/unitree_rl_mjlab && cd ~/unitree_rl_mjlab/ros2
./setup_external.sh                      # src/external 이 없다면

source /opt/ros/humble/setup.bash        # ★ 다른 워크스페이스는 source하지 말 것
colcon build --merge-install --packages-select \
    cyclonedds rmw_cyclonedds_cpp obstacle_detector \
    dpcbf_viz_msgs dpcbf_plot_client g1_perception_bringup

cat > ~/.g1_viz_env <<'EOF'
export G1_VIZ_DOMAIN_ID=7          # Computer 2와 동일
export G1_VIZ_IFACE=wlp2s0         # Computer 2로 나가는 NIC
#export G1_VIZ_PEER=192.168.50.20  # static 모드일 때 Computer 2 IP
EOF
```

> `colcon build` 전에 다른 워크스페이스를 source해 두면 colcon이 그것을
> underlay로 **체인**해 버려서, 실기에서 그 경로가 없으면 조용히 깨진다.
> Computer 3에서는 `/opt/ros/humble`만 source한 상태로 빌드할 것.
> `ros-humble-rmw-cyclonedds-cpp`를 apt로 깔아도 되지만(1.3.4), 그러면
> 저장소가 고정한 버전(0.10.2)과 달라지므로 위 방식을 권장한다.

### 4.1 Computer 2 (SSH 접속해서 실행)

SSH가 끊기면 launch와 bag이 SIGHUP으로 죽으므로 **반드시 tmux 안에서**
실행한다. **pane을 새로 만들 때마다 env 블록을 다시 붙여넣는다** (새 pane은
tmux server 시작 시점의 환경을 상속하므로 export가 따라오지 않는다 —
"토픽 이름은 보이는데 data가 없다"의 최대 원인).

```bash
ssh -o ServerAliveInterval=15 -o ServerAliveCountMax=4 <user>@<onboard-pc>
tmux new -s g1                 # 재접속: tmux attach -t g1
```

**env 블록 (모든 pane 공통)**

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_rl_mjlab/ros2/install/setup.bash
source ~/unitree_rl_mjlab/ros2/install/share/g1_perception_bringup/env/viz_env_computer2.sh multicast
#   multicast 안 되는 망이면  ... viz_env_computer2.sh static   (PEER 변수 필요)
#   G1_SENSOR_IFACE가 설정돼 있으면 dual-NIC XML이 자동 선택된다
export SESSION=~/unitree_rl_mjlab/ros2/evidence/hardware/$(date +%F)/s1 && mkdir -p "$SESSION"
printenv RMW_IMPLEMENTATION ROS_DOMAIN_ID CYCLONEDDS_URI    # 매 pane에서 확인
```

| pane | 명령 |
|---|---|
| 0 | **preflight** → `ros2 run g1_perception_bringup g1_hw_preflight.sh` (PASS 아니면 중단) |
| 0 | **스택** → `ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py enable_plot_bridge:=true plot_publish_rate:=30.0` |
| 1 | **read-out** → `ros2 run g1_perception_bringup hw_obstacle_watch.py` |
| 2 | **bag** → `ros2 run g1_perception_bringup hw_record.sh` (또는 launch에 `record:=on`) |
| 3 | **점검** → 아래 rate 확인 |

```bash
# pane 3: 살아 있어야 하는 것 (hw_diagnostics의 기대값)
ros2 topic hz /livox/lidar     # 10 Hz
ros2 topic hz /odom            # 100 Hz  (DLIO)
ros2 topic hz /scan            # 10 Hz
ros2 topic hz /obstacles_safe  # 10 Hz
ros2 topic echo /diagnostics --once     # 전 항목 OK 인지
```

`g1_perception_dpcbf.launch.py`는 `g1_perception_hardware_only.launch.py`를
**그대로 include**하는 superset이다 — perception-only 격리 보장(actuation
경로 없음, command 토픽 없음)은 hw_offline_gates가 매 빌드 계속 검사한다.
`enable_plot_bridge`/`plot_publish_rate`는 `UNITREE_DPCBF_PLOT`/
`UNITREE_DPCBF_PLOT_RATE`로 export되며, **DPCBF control binary가 생기기
전까지는 아무 효과가 없다**(위 경고 참조).

### 4.2 Computer 3 (노트북에서 실행)

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_rl_mjlab/ros2/install/setup.bash
source ~/unitree_rl_mjlab/ros2/install/share/g1_perception_bringup/env/viz_env_computer3.sh multicast
#   Computer 2와 같은 모드(multicast|static)를 쓸 것

# 1) 먼저 discovery 확인 — 여기서 안 보이면 GUI를 띄워도 소용없다
ros2 topic list | grep -E "odom|obstacles_safe"
ros2 topic hz /odom              # 100 Hz 근처면 링크 정상
ros2 topic hz /obstacles_safe    # 10 Hz

# 2) 플로팅 클라이언트
ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py
#   옵션: window_s:=60.0  gui_rate_hz:=20.0  stale_after_s:=1.0
#         backend:=matplotlib   (pyqtgraph가 없거나 문제될 때)
```

화면: 좌측 top-down(로봇 위치·heading·trail, 장애물 원 + 속도 방향,
좌상단 source별 stale banner), 우측 시계열 5단. 실기에서는 `/dpcbf/plot`
관련 요소만 비어 있고 나머지는 라이브로 갱신된다. 소스가 끊기면 해당 줄이
빨간 `STALE n.ns`로 바뀌며 **GUI는 계속 갱신된다**(멈춘 창은 아무것도
말해주지 않으므로 일부러 이렇게 만들었다).

Computer 3은 subscribe 전용이며 전부 BestEffort다 — 노트북을 끄든, Wi-Fi가
끊기든, 클라이언트가 죽든 Computer 2의 파이프라인에는 영향이 없다.

### 4.3 안 보일 때 (순서대로)

| 증상 | 확인 |
|---|---|
| `ros2 topic list`에 아무것도 없음 | 양쪽 `printenv ROS_DOMAIN_ID CYCLONEDDS_URI RMW_IMPLEMENTATION` 일치? tmux pane마다 env 재적용했나? |
| 토픽 이름은 보이는데 data 0 | multicast 차단 → 양쪽 `static` 모드로 전환(`G1_VIZ_PEER` 필요). `ros2 multicast receive` / `send`로 판정 |
| Computer 2에서만 안 보임 | 센서망/시각화망이 다른 NIC → `G1_SENSOR_IFACE` 설정해 dual-NIC XML 사용. `CYCLONEDDS_URI`는 프로세스당 하나 — 기존 `~/cyclonedds.xml`과 **병합되지 않는다** |
| 방화벽 | UDP 7400–7500 (도메인에 따라 이동) 열려 있는지 |
| 클라이언트가 기동하다 죽음 | `librmw_cyclonedds_cpp.so` 없음 → §4.0의 Computer 3 빌드 목록 재확인 |
| 장애물 원만 안 보임 | `obstacle_detector` 미빌드 → 클라이언트 로그에 그렇게 찍힌다 |

## 5. CycloneDDS 설정

XML은 `g1_perception_bringup/config/cyclonedds/`에 있고 값은 전부
env 치환(`${G1_...}`)이다 — IP/인터페이스 하드코딩 없음.

| 파일 | 용도 |
|---|---|
| `viz_multicast.xml` | 단일 NIC + multicast discovery (기본) |
| `viz_static_peers.xml` | 단일 NIC + multicast 차단 망 (peer IP 명시) |
| `c2_dual_nic_multicast.xml` | Computer 2 전용: 센서 NIC + 시각화 NIC 동시 참여 |
| `c2_dual_nic_static_peers.xml` | 위와 같되 multicast 차단 (C1/C3 peer 명시) |
| `localhost.xml` | **개발용 loopback 전용.** env 스크립트가 `G1_VIZ_ALLOW_LOCALHOST=1` 없이는 거부 |

`viz_env_computer{2,3}.sh [multicast|static|localhost]`가
`RMW_IMPLEMENTATION`/`ROS_DOMAIN_ID`/`CYCLONEDDS_URI`를 설정하고, NIC 존재
여부와 필수 변수를 검사한 뒤 선택 결과를 출력한다. tmux pane마다 다시
source해야 한다(perception runbook과 같은 이유).

주의: `CYCLONEDDS_URI`는 프로세스당 한 값이다. Computer 2에서 기존
`~/cyclonedds.xml`(센서 NIC 고정)을 쓰고 있었다면, 시각화까지 하는 세션은
dual-NIC XML **하나로 교체**해야 한다 — 두 XML은 병합되지 않는다.

## 6. 검증

### 6.1 로봇 없이 (이 순서대로 이미 수행됨 — 아래 §6.2)

```bash
# 1) 단위/통합 테스트
colcon test --merge-install --packages-select dpcbf_ros_adapter dpcbf_plot_client

# 2) localhost end-to-end (한 컴퓨터)
export G1_VIZ_ALLOW_LOCALHOST=1 G1_VIZ_DOMAIN_ID=42
source .../viz_env_computer3.sh localhost
ros2 run dpcbf_plot_client synthetic_dpcbf_publisher &
ros2 topic hz /dpcbf/plot          # ≈ 30 Hz
ros2 topic info -v /dpcbf/plot     # BEST_EFFORT / VOLATILE
ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py
# publisher를 죽이면 → banner STALE 확인
```

synthetic source는 로봇이 원을 걷고 obstacle 하나가 주기적으로 경로를
가로지르는 시나리오를 만들어 intervention/min_h dip까지 화면에서 확인할 수
있게 한다. `stop_after_s:=<s>`로 stale 전환을 자동 검증할 수 있다.
(주의: synthetic source는 `/odom`을 publish하므로 실제 스택 옆에서 절대
띄우지 말 것 — `g1_perception_dpcbf.launch.py`는 `synthetic:=on stack:=hw`
조합을 거부한다.)

### 6.2 검증 기록 (2026-08-06, dev 머신)

* `dpcbf_ros_adapter` gtest 32/32 (viz publisher: 내용 일치, 30↔20 Hz
  decimation, control 정지 시 publish 중단, 20 k회 Push 최악 지연 < 5 ms
  bound, yaml 로더 검증 포함), `dpcbf_plot_client` pytest 8/8,
  `hw_offline_gates` 283/283.
* localhost E2E 15/15: 수신(30 Hz 실측 30.00), GUI 렌더, stale 전환,
  스크린샷 저장.
* 격리: 클라이언트 attach 전/중/`kill -9` 후 `/dpcbf/plot` 30.0 Hz 불변.
* simulate ROS2 빌드( `build_ros2` ) 링크/빌드 정상, `dpcbf_safety_filter_test`
  통과 (T1/boundary는 capture fixture 없는 머신이라 원래 SKIP).
* 기존 실패 유지 항목: bringup의 fixture-bag 기반 launch test 4종은 이
  머신에서 변경 전(8/4)부터 실패하던 것(bag 부재 + harness teardown flake)
  으로 이번 변경과 무관.

### 6.3 풀 walking 리허설 (한 컴퓨터, 실험과 가장 가까운 구성)

g1_ctrl policy + 시뮬레이터(DPCBF estimated, scripted bring-up/walking) +
source_sim + perception 전체를 띄워두고 plot client를 붙이는, Computer 2/3
세션의 벤치 아날로그:

```bash
# 터미널 1 — 전체 스택 (Ctrl-C로 전부 정리)
ros2/src/g1_perception/g1_perception_bringup/test/walk_plot_session.sh W2 estimated
# W1(정적 sparse) W2(교차 sparse) W3(20개 swarm) W4(90개 arena) / oracle|estimated
# 창 없이 돌리려면 WALK_HEADLESS=1

# 터미널 2 — 스크립트가 출력해주는 env 블록 붙여넣기 후
ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py
```

타임라인(sim s): 15.5 FixStand → 21.5 policy → 34–40 band 하강 → 40+ 보행.
2026-08-06 headless 검증: `/dpcbf/plot` 30 Hz + `/odom` 99 Hz +
`/obstacles_safe` 10 Hz 동시 수신, mode=estimated fresh(age ~0.08 s 톱니 =
10 Hz 스트림의 정상 형태), 실제 intervention 에피소드와 min_h < 0 dip까지
클라이언트 렌더 확인.

참고: `config/walk_profile.txt`는 runbook §6.2 원문대로 복원된 파일이다
(문서에만 있고 커밋된 적이 없었음). 또한 walk 계열 harness 4종의 shadow
tree에 `ros2/` 심링크가 추가되었다 — Phase 5C부터 main.cc가
`dpcbf_ros_adapter.yaml`(및 `plot_bridge:`)을 exe 경로 기준으로 찾으므로 이
링크 없이는 시뮬레이터가 시작을 거부한다.

### 6.4 실제 2대 배포에서 확인할 것

1. 두 컴퓨터에서 `viz_env_*` source 후 `ros2 multicast receive/send`로
   multicast 가능 여부 판정 → 안 되면 static 모드로.
2. `ros2 topic hz /dpcbf/plot` (Computer 3에서): Wi-Fi에서 25–30 Hz면 정상.
   `/odom`, `/obstacles_safe`도 각각 hz 확인.
3. discovery에 안 보이면: 두 쪽 `ROS_DOMAIN_ID` 일치, `printenv
   CYCLONEDDS_URI RMW_IMPLEMENTATION`, 방화벽(UDP 7400–7500), NIC 이름.
4. Computer 2에서 control binary의 `/dpcbf/status` age 및 loop 주기가
   클라이언트 접속 전후 동일한지 (개입 없음 재확인).
5. GUI latency 곡선을 쓸 거면 두 컴퓨터 chrony/NTP 동기 여부 기록.
6. Wi-Fi 대역: `/dpcbf/plot` 30 Hz ≈ 수십 kB/s 수준. `/odom`·
   `/obstacles_safe`를 합쳐도 raw cloud 없이 수백 kB/s 미만이어야 정상.

## 7. 안전/격리 보증 (요약)

* control seam의 추가 비용은 `Push()` 한 번 = try_lock + ≤ ~1.3 kB memcpy.
  contention 시 그 tick의 샘플은 버려질 뿐 대기하지 않는다(카운터로 관측
  가능). QP 결과, command 경로, T1 byte-equivalence 대상 코드는 변경 없음.
* `dpcbf/` core는 손대지 않았다 (D3 유지). ROS 의존성은 전부
  `dpcbf_ros_adapter`(원래 rclcpp를 갖던 유일한 seam 패키지)에 있다.
* plot bridge를 꺼도(`enabled: false` / `UNITREE_DPCBF_PLOT=0`) seam은
  publisher를 아예 만들지 않는다.
* Computer 3 클라이언트는 subscribe 전용이며 BestEffort라 publisher에
  역압을 만들 수 없다. 죽거나(SIGKILL 포함) 네트워크가 끊겨도 Computer 2는
  변화 없음(실측 30.0 Hz 유지).
