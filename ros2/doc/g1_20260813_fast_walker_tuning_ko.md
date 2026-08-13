# G1 현장 매뉴얼 — 2026-08-13: 빠르게 걷는 사람을 위한 perception tuning

**대상**: 2026-08-13 세션. 어제(08-12)는 pipeline이 end-to-end로 동작했지만,
**빠르게 걷는 사람이 안정적인 원으로 변환되지 않았습니다.** 천천히 걸을 때는
동작하지만, 빠르게 걸으면 장애물을 놓치거나 원이 깜빡이고 튀며 사람보다 뒤처집니다.

**이 문서는 parameter tuning 매뉴얼입니다.** 전체 chain의 각 parameter가
물리적으로 무엇을 하는지, 어떤 증상을 해결하는지, 지나치게 조정하면 무엇이
망가지는지, 변경 결과를 화면에서 30초 안에 확인하는 방법을 설명합니다.

Tuning에 사용할 계측 도구는 어제 작성한 **`dpcbf_scan_view`**(commit
`d049fb7`, `f9da9c7`)입니다. 원을 fitting할 때 사용한 `/scan`을 로봇 자체
frame에서, 움직이지 않는 고정 축 위에 그립니다. "fit이 나쁘다"와 "fit할 입력
자체가 없다"를 구분할 수 있는 유일한 화면입니다.

> **Network link**(Fast DDS, `ROS_DOMAIN_ID`, multicast, peers mode,
> troubleshooting)에 관한 모든 내용은
> [`g1_fastdds_field_manual_ko.md`](g1_fastdds_field_manual_ko.md)에 있습니다.
> 이 문서에서는 terminal을 띄우는 데 필요한 최소 내용만 반복하며, 어느
> machine에서도 **`~/.bashrc`를 수정하지 않습니다.** 각 terminal에서 환경을
> 직접 source하십시오(§1.3).

---

## 목차

- [0. 빠르게 걷는 사람을 놓치는 이유 — failure chain](#0-빠르게-걷는-사람을-놓치는-이유--failure-chain)
- [1. Terminal과 환경 설정 (.bashrc 미사용)](#1-terminal과-환경-설정-bashrc-미사용)
- [2. Comp2 준비](#2-comp2-준비-g1-onboard-foxy)
- [3. Comp3 준비](#3-comp3-준비-laptop-humble)
- [4. Link 확인 — 요약판](#4-link-확인--요약판)
- [5. Tuning에 사용하는 세 개의 창](#5-tuning에-사용하는-세-개의-창)
- [6. Run 0 — baseline 측정](#6-run-0--baseline-측정-가장-먼저-수행)
- [7. Tuning 본문](#7-tuning-본문)
- [8. Decision tree — 어느 stage를 조정할 것인가](#8-decision-tree--어느-stage를-조정할-것인가)
- [9. 복사·붙여넣기용 run sheet](#9-복사붙여넣기용-run-sheet)
- [10. Troubleshooting](#10-troubleshooting)
- [11. 기록할 항목](#11-기록할-항목)

---

## 0. 빠르게 걷는 사람을 놓치는 이유 — failure chain

Pipeline은 하나의 chain입니다. 빠르게 걷는 사람은 **일곱** 지점에서 사라질 수
있으며, 각 지점마다 해결 방법이 다릅니다. 이것이 §8의 decision tree가 필요한
이유입니다.

```
/livox/lidar  ──CropBox──▶ /points_self_filtered ──p2l──▶ /scan
      10 Hz                                        (height band!)
                 ──extractor──▶ /raw_obstacles ──tracker──▶ /tracked_obstacles
                   (grouping)                     (association + KF)
                                       ──safety filter──▶ /obstacles_safe
                                          (gating + inflation)
```

| # | 지점 | 빠른 움직임이 일으키는 현상 | Parameter (§7 stage) |
|---|---|---|---|
| ① | Livox frame accumulation | `/livox/lidar` frame 하나에는 **100 ms 동안의 점이 누적**됩니다. 사람이 1.5 m/s로 걸으면 이동 방향을 따라 **0.15 m** 번집니다. Fitting된 원은 커지고, 중심은 실제 위치보다 번짐 길이의 절반 정도 뒤처집니다 | `publish_freq` (**B**) |
| ② | Height band (`min/max_height`) | 어제 사용한 band는 대략 **무릎에서 골반 높이**입니다. 바로 **다리가 흔들리는** 구간입니다. 보행 중 다리는 최대 약 0.7 m까지 벌어지고, 번갈아 서로를 가리며, 몸통 속도의 최대 **두 배**로 움직입니다. Extractor에는 frame마다 나타났다 사라지는 cluster가 1개 또는 2개로 보입니다 | `min_height`/`max_height` (**A**) |
| ③ | Point starvation | 얇은 band와 sparse non-repetitive Livox pattern이 겹치면 3 m 거리의 사람에게서 **5개 미만의 return**만 남는 경우가 많습니다. 그러면 `min_group_points: 5`가 아무 메시지 없이 해당 group을 버립니다 | `min_group_points`, band 두께 (**A/C**) |
| ④ | Grouping / splitting | 번지고 두 다리로 나뉜 cluster의 내부 간격이 `max_group_distance`를 넘음 → 작은 fragment로 분리됨 → 각 fragment가 `min_group_points` 미만이 됨 → 폐기됨 | `max_group_distance`, `max_split_distance`, `max_merge_*` (**C**) |
| ⑤ | Association | **새로 생성된** track의 초기 속도는 0이므로 gate가 두 frame 사이의 실제 변위에 그대로 적용됩니다. 1.5 m/s에서는 0.15 m, 2.5 m/s에서는 0.25 m인데 gate는 **0.30 m**에 불과하고 radius mismatch penalty까지 더해집니다. 여기서 놓치면 track이 한 번도 promote되지 않습니다. 즉 `/tracked_obstacles`가 비고, 따라서 `/obstacles_safe`도 비게 됩니다 | `min_correspondence_cost` (**D**) |
| ⑥ | KF lag | `measurement_variance: 1.0`은 1-σ가 **1 m인** measurement를 뜻합니다. 파일 자체에도 상속받은 잘못된 값이라고 표시되어 있습니다. Steady-state gain은 대략 **K ≈ 0.25**이고 step을 받아들이는 데 약 0.4초가 걸립니다. 한 번 횡단하는 시간이 약 2초이므로 **횡단 전 구간이 filter transient**가 됩니다 | `measurement_variance`, `process_rate_variance` (**D**) |
| ⑦ | Safety gating | `max_age: 0.30`은 extractor가 3 frame을 놓치면 **message 전체를 폐기**합니다. `v_max_obstacle: 1.5`는 빠르게 걷는 사람의 속도를 clamp합니다 | `max_age`, `v_max_obstacle` (**E**) |

> **오늘 가장 효과가 큰 단일 변경은 ②입니다.** Projection band를 흔들리는
> 다리에서 몸통으로 옮기십시오. §7-A를 먼저 수행하고, 다른 항목을 건드리기 전에
> 다시 측정하십시오.

---

## 1. Terminal과 환경 설정 (`.bashrc` 미사용)

### 1.1 이번 세션에서 사용할 terminal

| | Machine | 용도 |
|---|---|---|
| **T1** | Comp3 → Comp2로 SSH | perception stack 실행, 각 tuning iteration마다 정지·재시작 |
| **T2** | Comp3 → Comp2로 SSH | rate 확인 + `hw_obstacle_watch.py`(수치 확인) |
| **T3** | Comp3 → Comp2로 SSH | bag recording |
| **T4** | Comp3 → Comp2로 SSH | **tuning terminal**: YAML 편집 → `colcon build` → `config_diff` |
| **T5** | Comp3 자체 | **`dpcbf_scan_view`** — 주 계측 도구 |
| **T6** | Comp3 자체 | **RViz2** — raw point cloud + 2-D laser scan |
| **T7** | Comp3 자체 | link 확인 / `dpcbf_plot_client`(선택 사항, odom-frame view) |

T1–T4는 Comp2, T5–T7은 Comp3에서 실행합니다. 새 terminal은
`Ctrl`+`Alt`+`T`, 새 tab은 `Ctrl`+`Shift`+`T`, tab 전환은
`Alt`+`1`, `Alt`+`2`, …입니다.

**모든 terminal에서 첫 명령으로 `hostname`을 입력하십시오.** 현장 문제의
절반은 잘못된 machine에서 명령을 실행해서 생깁니다.

### 1.2 환경 파일(machine별로 한 번만 작성)

**Comp3 (laptop, user `dyros`)** — 이미 가지고 있는 block:

```bash
cd ~
cat > ~/.g1_net_env <<'EOF'
export ROS_DOMAIN_ID=7                 # ★ Comp2와 반드시 같아야 함
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
export G1_WS=/home/dyros/unitree_rl_mjlab/ros2      # ★ Comp2와 다른 경로
#export G1_PEER_IP=192.168.123.164     # Comp2 IP. peers mode에서만 사용
EOF
```

**Comp2 (G1 onboard, user `unitree`)** — 같은 파일이지만 **`G1_WS`가 다릅니다**:

```bash
# 먼저 실제 경로를 확인하십시오. 추측하지 마십시오.
cd /home/unitree/dyros_ws/sanghyuk_ws/unitree_rl_mjlab/ros2 && pwd
# 실패하면 다음을 실행합니다.
find /home -maxdepth 6 -name deps.repos -path "*/ros2/*" 2>/dev/null
```

```bash
cd ~
cat > ~/.g1_net_env <<'EOF'
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
export G1_WS=/home/unitree/dyros_ws/sanghyuk_ws/unitree_rl_mjlab/ros2   # ★ 방금 확인한 경로
#export G1_PEER_IP=<Comp3 IP>
EOF
```

| Variable | Comp2 | Comp3 | 조건 |
|---|---|---|---|
| `ROS_DOMAIN_ID` | `7` | `7` | **반드시 동일** |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | `rmw_fastrtps_cpp` | **반드시 동일** |
| `ROS_LOCALHOST_ONLY` | `0` | `0` | 둘 다 0 |
| `G1_WS` | `/home/unitree/dyros_ws/sanghyuk_ws/…/ros2` | `/home/dyros/unitree_rl_mjlab/ros2` | **서로 다름** |
| `G1_PEER_IP` | Comp**3** IP | Comp**2** IP | **서로 반대편**, peers mode에서만 사용 |

### 1.3 ★ **모든** terminal에 붙여 넣을 block

어느 machine에서도 `~/.bashrc`에 아무것도 추가하지 않으므로, 새 terminal에는
아래 내용을 붙여 넣기 전까지 ROS 환경이 **없습니다.** 이를 잊는 것이 "topic
list가 비어 있다"는 문제의 가장 흔한 원인입니다.

**Comp2(T1–T4), 4줄:**

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

**Comp3(T5–T7), 4줄:**

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

두 machine에서 모두 동작하는 한 줄 확인 명령:

```bash
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY | tr '\n' ' '; echo; ls "$G1_WS/deps.repos"
```

예상 결과는 `7 rmw_fastrtps_cpp 0`과 `deps.repos` 경로입니다. `ls`가
*No such file or directory*를 출력하면 `G1_WS`가 잘못된 것입니다. 다른 작업을
하기 전에 수정하십시오. 그렇지 않으면 `cd "$G1_WS"`가 조용히 실패하고,
`source install/setup.bash`가 엉뚱한 directory에서 실행됩니다("build했는데
package가 없다").

> **Comp2에 SSH로 접속**(Comp3 terminal에서):
> ```bash
> ssh -o ServerAliveInterval=15 -o ServerAliveCountMax=4 unitree@<Comp2 IP>
> ```
> Laptop의 화면 꺼짐과 자동 suspend를 끄고 lid를 닫지 마십시오. Laptop이
> sleep 상태가 되면 SSH session 네 개가 동시에 끊어집니다.

---

## 2. Comp2 준비 (G1 onboard, Foxy)

**T1**에서 4줄 block을 실행한 뒤:

```bash
cd "$G1_WS"
git pull
source /opt/ros/foxy/setup.bash
colcon build --packages-select g1_perception_bringup
```

이 rebuild는 몇 초면 끝나며 `config/*.yaml`을 `install/share/`로 복사합니다.
**`ros2 launch`는 편집한 source file이 아니라 설치된 사본을 읽습니다.** 이것이
§7.1 전체의 핵심입니다.

이 machine에서 workspace를 한 번도 build하지 않았다면 Fast DDS 매뉴얼 §2-4의
15-package build를 사용하십시오.

당일 첫 run 전에 preflight가 반드시 PASS해야 합니다.

```bash
./src/g1_perception/g1_perception_bringup/scripts/g1_hw_preflight.sh
echo "EXIT=$?"
```

---

## 3. Comp3 준비 (laptop, Humble)

**T5**에서 4줄 block을 실행한 뒤:

```bash
cd "$G1_WS"
git pull
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-select \
    obstacle_detector dpcbf_viz_msgs dpcbf_plot_client
```

오늘은 `git pull`이 중요합니다. `dpcbf_scan_view`와 `roll = pi` 수정
(`f9da9c7`)이 어제 저녁에 반영되었습니다. 수정이 없으면 **모든 원이 로봇 전방
축을 기준으로 대칭 반전**되지만 겉보기에는 그럴듯해서 알아채기 어렵습니다.

확인:

```bash
source install/setup.bash
ros2 pkg executables dpcbf_plot_client
```

예상 결과는 세 항목이며 세 번째가 오늘 사용할 도구입니다.

```
dpcbf_plot_client dpcbf_plot_client
dpcbf_plot_client synthetic_dpcbf_publisher
dpcbf_plot_client dpcbf_scan_view
```

Laptop을 새로 설정하는 경우 필요한 apt package:

```bash
sudo apt-get install -y python3-matplotlib python3-pyqtgraph python3-pyqt5 \
                        python3-pyqt5.qtopengl ros-humble-laser-geometry
```

---

## 4. Link 확인 — 요약판

전체 절차는 Fast DDS 매뉴얼 §4에 있습니다. Comp2에서 stack이 이미 실행 중일
때 tuning을 시작하기 전 최소 확인 절차는 다음과 같습니다. **Comp3의 T7**에서:

```bash
ros2 daemon stop
ros2 topic hz /scan --no-daemon                # ~10
ros2 topic hz /obstacles_safe --no-daemon      # ~10
```

두 topic이 laptop에 도착하면 §5의 모든 기능이 동작합니다. 도착하지 않으면
**멈추고 link부터 고치십시오**(Fast DDS 매뉴얼 §6-1). 끊어진 link를 통해
tuning하면 parameter가 아니라 network에 관한 잘못된 결론을 얻게 됩니다.

---

## 5. Tuning에 사용하는 세 개의 창

### 5.1 `dpcbf_scan_view` — 주 계측 도구(T5, Comp3)

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run dpcbf_plot_client dpcbf_scan_view
```

Option을 함께 사용할 때는 오늘 아래 형식으로 실행합니다.

```bash
ros2 run dpcbf_plot_client dpcbf_scan_view --ros-args \
    -p range:=4.0 -p scan_history:=5
```

| Parameter | Default | 의미 / 변경 시점 |
|---|---|---|
| `range` | `5.0` | 고정 축의 반폭[m]. 사람이 창 안에서 크게 보이도록 횡단 거리보다 1 m 크게 설정합니다 |
| `scan_history` | `1` | 최근 N개 scan을 겹쳐 그리고 오래된 scan일수록 흐리게 표시합니다. **오늘은 `5`를 사용합니다.** 사람의 움직임이 점의 궤적으로 보여서 *scan noise*와 *fit noise*를 한눈에 구분할 수 있습니다 |
| `target_frame` | `base_link` | 모든 항목을 그릴 frame입니다. `odom`은 world view와 비교할 때 사용합니다. `''`는 "/scan이 들어오는 frame 그대로"라는 뜻이며 오늘은 `mid360_link`입니다. **이 frame은 위아래가 뒤집혀 있으므로** 화면도 대칭 반전됩니다. 사용하지 마십시오 |
| `obstacle_topics` | raw, tracked, safe | 서로 다른 style로 그릴 세 pipeline stage |
| `scan_topic` | `/scan` | — |
| `stale_after_s` | `1.0` | 이 age를 넘으면 banner가 빨간색으로 바뀝니다 |

**화면 읽는 법** — 진단에 필요한 전부입니다.

| 화면 표시 | Source | 의미 |
|---|---|---|
| 밝은 파란 점 | `/scan`, 최신 frame | **Extractor에 실제로 들어온 값입니다.** 여기에 사람이 없으면 downstream parameter로는 해결할 수 없습니다 |
| 흐린 파란 점 | 이전 N개 scan | 움직임의 궤적. 여기서 보이는 jitter는 *sensor/projection noise*입니다 |
| **회색 점선 원** | `/raw_obstacles` | extractor가 frame마다 만든 fit |
| **주황색 원 + uid + r** | `/tracked_obstacles` | tracker의 KF output. Label은 `uid r=<true_radius>`입니다 |
| **빨간색 원** | `/obstacles_safe` | gating + inflation 이후의 결과로, DPCBF가 사용하게 될 값입니다 |
| 원에서 뻗은 점선 | velocity | 중심이 **1초 뒤** 도달할 위치입니다 |
| 원점의 초록 점과 선 | robot | (0,0)에 고정되며 전방은 +x, 좌측은 +y입니다 |
| 좌상단 banner | age | 초록색 = fresh, 빨간색 = `STALE`/`NO DATA` |

**네 가지 색으로 failure chain 전체를 볼 수 있습니다.** 점은 있지만 회색 원이
없으면 extractor(§7-C), 회색은 있지만 주황색이 없으면 tracker(§7-D), 주황색은
있지만 빨간색이 없으면 safety filter(§7-E), 점 자체가 없으면
projection(§7-A) 문제입니다.

> Velocity 점선은 speed read-out이기도 합니다. 점선의 길이(단위 **m**)가
> estimated speed(m/s)입니다. 1.5 m/s로 걷는 사람이라면 진행 방향으로 약
> 1.5 m 길이의 점선이 보여야 합니다. 방향이 frame마다 흔들리면 tracker가
> track을 이어 가는 것이 아니라 계속 새로 만들고 있는 것입니다.

### 5.2 `hw_obstacle_watch.py` — 수치 확인(T2, Comp2)

같은 내용을 **base_link** 기준 console table로 표시하며 `radius`와
`true_radius`를 모두 출력합니다.

```bash
ros2 run g1_perception_bringup hw_obstacle_watch.py
ros2 run g1_perception_bringup hw_obstacle_watch.py --ros-args \
    -p rate:=2.0 -p json:=$SESSION/tuning_watch.jsonl
```

*수치를 기록해야 할 때* 사용하십시오. 화면의 원은 동작 여부를, 이 도구는 어느
정도로 동작하는지를 알려 줍니다. `json:` parameter는 frame마다 JSON 한 줄을
추가하므로 "run 3이 run 2보다 나았다"는 판단의 근거가 됩니다.

### 5.3 RViz2 — raw point cloud와 2-D laser scan(T6, Comp3)

**"사람이 height band 안에 들어오기나 하는가?"에 답하는 화면입니다.**

Repository에는 필요한 두 display가 들어 있는 robot-frame layout이 이미
있습니다. Source tree의 일반 파일이므로 Comp3에서 사용하기 위해
`g1_perception_bringup`을 build할 필요는 없습니다.

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

rviz2 -d "$G1_WS/src/g1_perception/g1_perception_bringup/rviz/perception_robot_frame.rviz"
```

| Layout의 display | Topic | 참고 |
|---|---|---|
| `LivoxCloud` | `/livox/lidar` | **Raw 3-D cloud**. Commit된 layout에서는 처음에 *disabled* 상태이므로 checkbox를 켜십시오 |
| `Scan (extractor input)` | `/scan` | Projection 이후 height band에 해당하는 2-D scan |
| `Odom` | `/odom` | |
| `RawObstacles` / `TrackedObstacles` / `SafeObstacles` | marker relay | **계속 비어 있는 것이 정상입니다.** 이 relay들은 hardware stack에 포함되지 않은 `viz.launch.py`에서만 실행됩니다. 원은 `dpcbf_scan_view`로 확인하십시오 |

Fixed Frame은 `base_link`입니다.

**직접 구성하려는 경우**(또는 config를 불러오지 못하는 경우):

```bash
rviz2
```

1. **Global Options → Fixed Frame**: `base_link`(또는 아래 설명의 `mid360_link`)
2. **Add → By topic → `/livox/lidar` → PointCloud2**
   - **Topic → Reliability Policy: `Best Effort`** ← 반드시 설정하십시오.
     Livox driver는 *Reliable, depth 256*로 publish합니다. Wi-Fi에서 Reliable
     subscriber를 사용하면 middleware가 0.5 MB message를 재전송하느라
     `/odom`이 밀릴 수 있습니다.
   - Size (m): `0.02`, Style: `Points`
   - **Color Transformer: `AxisColor`, Axis: `Z`** — 화면에서 높이를 직접
     읽을 수 있게 해 주는 설정입니다.
3. **Add → By topic → `/scan` → LaserScan**
   - Reliability Policy: `Best Effort`, Style `Points`, Size `0.05`, 빨간색
4. CropBox self-filter가 제거한 점을 보려면 **Add →
   `/points_self_filtered` → PointCloud2**를 추가합니다.

> ⚠ **Bandwidth.** `/livox/lidar`는 대략 **5 MB/s**입니다(약 20,000 point ×
> 26 byte × 10 Hz). Wi-Fi에서는 link의 다른 모든 traffic과 경쟁합니다. 특정
> 질문을 확인할 때만 잠깐 켠 뒤 checkbox를 끄십시오. `/scan`은 frame당 약
> 4 kB여서 부담이 없으므로 계속 켜 두어도 됩니다.

> **Height band를 직접 측정하는 방법**: Fixed Frame을 `mid360_link`로 바꾸고
> Z축 `AxisColor`를 활성화한 다음 toolbar의 **Measure**로 바닥부터 사람의 한
> 점까지 클릭하십시오. 부호에 주의하십시오. Mid-360은 뒤집혀 장착되어
> `roll = π`이므로 `mid360_link`에서 **양의 z는 sensor 아래쪽**입니다. 바닥은
> `z ≈ +H`에 나타나며 `H`는 바닥에서 sensor까지의 높이입니다. 이것이 §7-A에
> 사용할 `H`를 가장 빠르게 얻는 방법입니다.

**대신 Comp2에서 실행하려면**(laptop에서 표시할 수 없는 경우)
`ssh -X unitree@<Comp2 IP>`로 접속하고 4줄 block을 붙여 넣은 뒤, §9-1의
launch 명령에 `use_rviz:=true`를 추가합니다.

---

## 6. Run 0 — baseline 측정 (가장 먼저 수행)

**Run 0을 확보하기 전에는 parameter를 변경하지 마십시오.** 그래야 오늘의 어느
변경이 효과가 있었는지 판단할 수 있습니다.

환경: 탁 트인 바닥, 서 있거나 stand에 고정된 robot, **2.5–3 m**의 일정한
거리에서 한 사람이 좌우로 횡단한 뒤 되돌아옵니다.

| Pass | 속도 | 대략적인 동작 |
|---|---|---|
| 1 | 느림 | 0.5 m/s — 산책하듯 걷기 |
| 2 | 보통 | 1.2 m/s |
| 3 | **빠름** | 1.8–2.0 m/s — 빠른 걸음, 현재 실패하는 조건 |
| 4 | **stop-and-go** | 빠르게 걷다가 2초 동안 완전히 멈춘 뒤 다시 걷기 |
| 5 | **robot 방향** | 4 m 거리에서 robot을 향해 1.5 m 지점까지 직진 |

각 pass에서 T5를 보고 **문자 하나**를 기록합니다.

| 문자 | 화면에서 본 현상 |
|---|---|
| **N** | 사람 위에 파란 점이 없음 — fit할 것이 없음 |
| **S** | 점이 약 5개 미만이거나 두 덩어리(다리)로 나뉨 |
| **R** | 회색 원이 나타나지만 계속 깜빡임 |
| **T** | 회색 원은 안정적이지만 **주황색 원이 없음**(track이 promote되지 않음) |
| **L** | 주황색 원은 있지만 이동 방향을 따라 **파란 점보다 뒤처짐** |
| **X** | 주황색 원은 정상이지만 **빨간색 원이 없음**(`/obstacles_safe`가 비어 있음) |
| **OK** | 횡단하는 내내 세 원이 모두 사람을 따라감 |

이 문자는 §8의 해당 stage로 바로 안내합니다. 각 pass의 screenshot을
`PrtSc`로 `$SESSION`에 저장하십시오. 다섯 장이 before/after 자료가 됩니다.

**Pass 3과 5에서는 bag도 recording하십시오**(§9-3). `/livox/lidar`가 들어
있는 bag은 저녁에 사람 없이도 **전체** downstream chain을 offline으로 다시
tuning할 수 있게 해 줍니다. `driver:=off lio:=off`로 replay하면 앉은 자리에서
§7-A/C/D/E의 모든 parameter를 sweep할 수 있습니다. 필요한 disk 용량만큼의
가치가 있습니다(약 5 MB/s, 30초 pass 하나에 약 150 MB).

---

## 7. Tuning 본문

### 7.1 ★ 실제 edit loop 동작 방식 (읽지 않으면 하루를 잃습니다)

**모르면 몇 시간을 낭비하게 되는 세 가지 사실:**

1. **Extractor와 tracker에는 `ros2 param set`이 적용되지 않습니다.** 이
   fork에서는 parameter-update service가 comment 처리되어 있습니다
   ([obstacle_extractor.cpp:50](../src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L50),
   [obstacle_tracker.cpp:53](../src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L53)).
   값은 node가 생성될 때 **한 번만** 읽습니다. `ros2 param set
   /obstacle_extractor min_group_points 3`은 `Set parameter successful`을
   반환하지만 **아무것도 바꾸지 않습니다.** 이 결과를 믿지 마십시오.
2. **`ros2 launch`는 편집한 YAML이 아니라 *설치된* YAML을 읽습니다.**
   `src/.../config/*.yaml`을 편집하고 곧바로 relaunch하면 **이전 값**으로
   실행되며, 이를 알려 주는 출력도 없습니다.
3. 다섯 perception node가 **하나의 container**에 component로 들어 있으므로
   변경할 때마다 DLIO를 포함한 stack 전체를 재시작해야 합니다. **Relaunch할
   때마다 robot을 3초 동안 움직이지 마십시오**(IMU calibration,
   `odom/imu/calibration/time: 3.0`).

**Comp2의 T4에서 수행할 loop:**

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/foxy/setup.bash
source install/setup.bash

nano src/g1_perception/g1_perception_bringup/config/<file>.yaml   # 편집

colcon build --packages-select g1_perception_bringup              # 약 5초: config를 install/로 복사
ros2 run g1_perception_bringup config_diff.py                     # 모든 줄이 IDENTICAL이어야 함
```

그런 다음 **T1**에서 `Ctrl-C`를 누르고 relaunch(§9-1)한 뒤 3초 동안 가만히
있다가 시험 중인 pass를 다시 수행합니다. **Iteration마다 stage 하나만**
바꾸십시오. 두 항목을 동시에 바꾸면 결과에서 아무것도 배울 수 없습니다.

> 당일 작업을 빠르게 하는 선택 사항: workspace를 한 번
> `colcon build --symlink-install --packages-select g1_perception_bringup`으로
> rebuild합니다. 설치된 config가 source file을 가리키는 symlink가 되어
> loop에서 `colcon build` 단계를 생략할 수 있습니다. 그래도 stack은 반드시
> 재시작해야 합니다. `config_diff.py`는 계속 통과합니다(동일 byte).

### 7.2 오늘 제안하는 시작값

먼저 **Stage A만 단독으로** 적용하고 다시 측정하십시오. 나머지는 §6에서 기록한
문자가 해당할 때만 적용합니다.

| Stage | File | Parameter | 08-12 | 제안값 | 이유 |
|---|---|---|---|---|---|
| **A** | `pointcloud_to_laserscan.yaml` | `min_height` | `0.2` | `-0.15` | band 상단을 sensor plane 위로 올림 |
| **A** | " | `max_height` | `0.7` | `0.65` | 바닥은 제외하면서 무릎 대신 허리→가슴 아래를 포함 |
| **C** | `obstacle_detector.yaml` | `min_group_points` | `5` | `3` | 얇은 band에서는 3 m 거리의 사람이 3–4 return만 만드는 경우가 많음 |
| **C** | " | `max_group_distance` | `0.10` | `0.15` | 한 cluster 안의 0.15 m motion smear를 허용 |
| **C** | " | `distance_proportion` | `0.01745` | `0.03` | 횡단 구간의 먼 쪽을 위한 range-scaled tolerance |
| **C** | " | `max_split_distance` | `0.20` | `0.25` | 번진 한 사람을 둘로 split하지 않게 함 |
| **C** | " | `max_merge_separation` | `0.20` | `0.35` | 두 다리를 한 obstacle로 merge |
| **C** | " | `max_merge_spread` | `0.20` | `0.30` | 위와 같음 |
| **D** | " | `min_correspondence_cost` | `0.3` | `0.5` | 2.0 m/s × 0.1 s = 0.20 m에 radius penalty를 더해도 gate 안에 들어와야 함 |
| **D** | " | `measurement_variance` | `1.0` | `0.04` | 1 σ를 1 m에서 0.2 m로 낮춰 KF lag를 줄임 |
| **D** | " | `process_rate_variance` | `0.03` | `0.10` | 사람의 속도 변화만큼 velocity state가 빠르게 변하도록 함 |
| **E** | `safety_obstacle_filter.yaml` | `max_age` | `0.30` | `0.50` | 3-frame detection gap 동안 유지 |
| **E** | " | `v_max_obstacle` | `1.5` | `2.5` | 빠르게 걷는 사람의 속도를 clamp하지 않음 |
| **B** | `livox_driver.yaml` | `publish_freq` | `10.0` | `20.0` | **최후 수단**. 함께 바꿔야 하는 세 항목은 §7-B 참고 |

---

### Stage A — projection band (`pointcloud_to_laserscan.yaml`)

**가장 중요한 stage입니다.**

#### 실제 동작

`pointcloud_to_laserscan`은 3-D cloud에서 **z가 `min_height`와
`max_height` 사이인 point만 남긴 뒤**, 그 수평 slab을 2-D `LaserScan`으로
투영합니다. Downstream에는 이 slab만 보입니다. Slab 밖의 사람은 이
pipeline에 존재하지 않는 것과 같습니다.

**부호 함정.** 어제 설정인 `target_frame: ''`에서는 transform을 적용하지
않으므로 `min_height`/`max_height`를 **`mid360_link` 기준**으로 측정합니다.
Mid-360은 뒤집혀 장착되어 이 frame에 `roll = π`가 적용됩니다
([`g1_mid360.xacro`](../src/g1_perception/g1_description/urdf/g1_mid360.xacro)).
따라서:

> `mid360_link`에서 **양의 `z` = sensor 아래쪽**입니다.

`H`를 바닥에서 sensor까지의 높이라고 할 때 실제 바닥 기준 높이와의 관계는
다음과 같습니다.

```
height above floor  =  H − z_mid360

band [min_height, max_height]  →  floor band [ H − max_height ,  H − min_height ]
```

**편집 전에 `H`부터 측정하십시오.** Robot이 실험 중 유지할 pose에서 줄자로
바닥부터 Mid-360 중심까지 재거나 RViz에서 읽습니다(§5.3에서 바닥은
`z_mid360 ≈ +H`). 아래 내용은 **H ≈ 1.20 m**를 가정하므로 실제 측정값으로
대체하십시오.

| 설정 | H = 1.20일 때 바닥 기준 band | 사람에게 닿는 부위 |
|---|---|---|
| 08-12: `0.20 … 0.70` | **0.50–1.00 m** | 무릎, 허벅지, 골반 — **흔들리는 다리** |
| 제안값: `-0.15 … 0.65` | **0.55–1.35 m** | 허벅지 → 허리 → 가슴 아래 — 하나로 이어진 몸통 |
| torso-only: `-0.15 … 0.35` | 0.85–1.35 m | 몸통만 포함. Point 수는 가장 적지만 원은 가장 깨끗함 |

**Band를 위로 옮기면 빠른 보행이 해결되는 이유.** 다리는 보행자에서 가장
빠르고 원과 가장 닮지 않은 부분입니다. Mid-stride에서는 폭 0.15 m인 두
물체가 최대 0.7 m 떨어져 있고 몸통의 두 배 속도로 움직이며 번갈아 서로를
가립니다. 반면 몸통은 0.3–0.4 m 크기의 단일 물체이고 정확히 보행 속도로
움직이며 split되지 않습니다. **이 pipeline의 circle model은 몸통을 표현하는
것이지 다리 한 쌍을 표현하는 것이 아닙니다.**

#### 넘지 말아야 할 두 한계

| 한계 | 규칙 | 위반 시 결과 |
|---|---|---|
| **수평선 위쪽 sensor FOV** | 뒤집혀 장착된 Mid-360은 자체 수평면 기준 위쪽 약 **7°**, 아래쪽 약 52°를 봅니다. 거리 `d` m의 point는 sensor보다 최대 `0.12 · d` 위에 있을 수 있습니다 | `min_height`가 `−0.12 · d_min`보다 더 작으면 근거리에서 빈 band가 됩니다. 1.5 m에서는 `min_height ≥ −0.18`이어야 합니다 |
| **바닥** | 바닥은 `z_mid360 ≈ +H`에 있습니다 | `max_height`가 `H`에 가까우면 scan이 **바닥 return의 원형 ring**으로 가득 차 extractor가 바닥에 원을 fitting합니다. `max_height ≤ H − 0.4`를 유지하십시오 |

#### Band 두께도 parameter입니다

Band가 두꺼우면 사람당 point가 늘어나 sparse Livox pattern에 강해지지만,
방 안의 의자·탁자·몸통 높이의 벽도 더 많이 들어옵니다. 탁 트인 횡단
환경에서는 **두꺼울수록 좋습니다.** 몸통 높이에 물체가 많은 방에서는 band를
좁히고 Stage C의 `min_group_points`로 보완하십시오.

> Band는 robot 몸통에 실린 **sensor 기준**입니다. Robot이 걸을 때 몸통
> pitch가 ±5°만 변해도 4 m 지점의 band 높이는 ±0.35 m 움직입니다. Robot
> 자체가 걷기 시작한 뒤에는 두꺼운 band가 필요한 또 하나의 이유입니다.

#### 대안: world frame에 band 배치

```yaml
target_frame: base_footprint   # min/max_height가 바닥 위 높이가 되며 부호가 정상
min_height: 0.70
max_height: 1.50
```

08-12 이전 설정이며 장기적으로는 *올바른* 해법입니다. 높이가 의미 그대로
표현되고 torso pitch가 band를 움직이지 않습니다. 다만 cloud마다 TF lookup
(`odom → base_footprint`)이 필요해 **DLIO가 projection path에 들어갑니다.**
어제 hanging test에서 이 설정을 끈 이유도 이것입니다. **오늘 DLIO가
정상이면 시도해 보십시오. 부호 함정 전체가 사라집니다.** `ros2 topic hz
/scan`으로 검증하고, `/points_self_filtered`는 계속 나오는데 `/scan`이 끊기면
TF 문제이므로 `target_frame: ''`로 돌아갑니다.

#### Stage A를 30초 안에 검증

1. T6(RViz): 사람이 `/livox/lidar`에 보이고, 이제 `/scan`에도 작은 arc로
   나타납니다.
2. T5(`dpcbf_scan_view`): 사람 위에 **5개 이상의 파란 점**이 compact한 한
   덩어리로 나타나며 횡단 내내 둘로 갈라지지 않습니다.
3. `ros2 topic hz /scan`은 계속 약 10 Hz입니다.

---

### Stage B — sensor rate (`livox_driver.yaml`) — *최후 수단*

#### 실제 동작

`publish_freq: 10.0`이면 driver가 Mid-360의 non-repetitive pattern을
**100 ms** 동안 누적해 `/livox/lidar` message 하나를 만듭니다. 이것이 failure
①의 motion smear 원인입니다. 1.5 m/s로 걷는 사람은 frame 하나 안에서
0.15 m 번져 fitting radius가 커지고 중심이 뒤로 끌립니다.

20 Hz에서는 번짐이 절반이 되지만 **frame당 point 수도 절반**이 되어 failure
③이 더 심해집니다. 실제 trade-off이므로 마지막에 시도합니다.

#### 시도한다면 네 항목을 모두 변경

```yaml
# livox_driver.yaml
publish_freq: 20.0

# pointcloud_to_laserscan.yaml
scan_time: 0.05          # 기존 0.1 — metadata이며 consumer가 사용

# obstacle_detector.yaml (두 node 모두)
sensor_rate: 20.0        # association cost model + fade-counter 크기
loop_rate: 20.0
```

`tracking_duration`은 `1.0`으로 유지합니다. 단위가 초이고 fade window는
`rate × duration`으로 계산되므로 자동으로 20 tick으로 바뀝니다.

#### 검증

```bash
ros2 topic hz /livox/lidar    # 20
ros2 topic hz /scan           # 20
ros2 topic hz /obstacles_safe # 20
top -b -n1 | head -15         # Comp2 CPU — container 부하가 약 두 배가 됨
```

그런 다음 §6의 pass 3을 다시 확인합니다. **`min_group_points` 때문에 사람이
다시 사라지면 값을 3(또는 2)으로 낮추십시오. 예상된 side effect입니다.** Bag
크기와 Wi-Fi 부하도 두 배가 됩니다.

---

### Stage C — extractor grouping (`obstacle_detector.yaml` → `obstacle_extractor`)

Extractor는 scan point를 순서대로 훑으며 **group → split → merge → circle
fit**의 네 단계를 수행합니다.

| Parameter | 08-12 | 실제 동작 | 해결하는 증상 | 지나치게 조정하면 |
|---|---|---|---|---|
| `min_group_points` | `5` | point 수가 이 값보다 적은 group은 fit하기 전에 **폐기** | **먼 거리에서 사람이 사라짐**(대표적인 silent drop) | 흩어진 return 2–3개가 phantom obstacle이 되고 noise에서 원이 생김 |
| `max_group_distance` | `0.10` | 연속한 두 scan point의 간격이 `max_group_distance + range · distance_proportion`보다 작으면 같은 group으로 묶음. 즉 **cluster-continuity threshold** | 번지거나 두 다리로 보이는 사람이 fragment로 찢어진 뒤 `min_group_points` 미만이 됨 | 사람이 **뒤쪽 벽/탁자와 merge**되고 fit이 `max_circle_radius`(0.60)를 넘어 전체가 폐기됨 |
| `distance_proportion` | `0.01745` (=1°) | 같은 threshold에서 **range-scaled** 부분. 4 m에서는 `4 × 0.01745 = 0.07 m`를 더함 | 횡단 구간의 **먼 쪽에서만** 생기는 fragment | 위와 같지만 먼 거리에서만 발생. 넉넉하게 설정해도 상대적으로 안전한 지점임 |
| `max_split_distance` | `0.20` | Grouping 후 chord에서 가장 멀리 벗어난 point의 deviation이 이 값을 넘으면 group을 split | 한 사람을 두 원으로 나누는 현상 | 실제로 다른 두 물체(사람 + 기둥)가 붙은 채 유지됨 |
| `max_merge_separation` | `0.20` | 두 segment가 이 값보다 가까우면 merge 후보가 됨 | **두 다리가 별도 obstacle로 남음** | 기둥 옆을 지나는 사람이 기둥과 merge됨 |
| `max_merge_spread` | `0.20` | 네 endpoint가 모두 merged line에서 이 거리 안에 있을 때만 merge | 위와 같음 | 위와 같음 |
| `max_circle_radius` | `0.60` | 이 값보다 큰 radius의 fit을 버리고 count하는 **hard drop** | — | 함부로 올리지 마십시오. §9.6에서 의존하는 sensing limit이며 `safety_obstacle_filter.yaml`에도 의도적으로 같은 값이 있습니다. **변경할 때는 두 파일을 함께 바꾸십시오.** |
| `radius_enlargement` | `0.17` | Fitting radius에 더하는 margin(`radius = true_radius + this`)으로, 일부만 보이는 arc의 bias를 보상 | 화면의 원이 너무 작음 | obstacle이 과도하게 inflate되어 DPCBF가 불필요하게 보수적으로 동작 |
| `circles_from_visibles` | `true` | 보이는 arc만으로 원을 fit | — | — |
| `frame_id` / `transform_coordinates` | `odom` / `true` | full TF로 scan frame을 변환해 extractor가 **odom** 기준으로 publish | — | **변경하지 마십시오.** `dpcbf_scan_view`와 `hw_obstacle_watch.py`가 모두 이 설정을 전제로 합니다 |

추측하지 않고 수치를 고를 수 있도록 grouping 계산식을 정리하면, 거리 `r`에서
인접한 두 return은 간격이 아래 값보다 작을 때 같은 물체로 묶입니다.

```
gap_max = max_group_distance + r · distance_proportion
```

| r | 08-12 (`0.10`, `0.01745`) | 제안값 (`0.15`, `0.03`) |
|---|---|---|
| 1 m | 0.12 m | 0.18 m |
| 3 m | 0.15 m | 0.24 m |
| 5 m | 0.19 m | 0.30 m |

사람 몸통 폭은 0.30–0.40 m이므로 제안값은 Livox pattern에 hole이 있더라도
5 m까지 몸통을 하나로 유지합니다.

#### Stage C 검증

T5에서 **회색 점선 원**이 frame마다(한 frame 건너 한 번이 아니라) 사람 위에
나타나며 `r =` 값이 약 **0.15–0.30**이어야 합니다. `r`이 0.5보다 커지면
사람이 다른 물체와 merge된 것입니다. `max_merge_separation`을 다시 낮추십시오.
점은 그대로인데 회색 원이 사라지면 `min_group_points` 또는
`max_circle_radius`에 의한 drop입니다.

---

### Stage D — tracker (`obstacle_detector.yaml` → `obstacle_tracker`)

Tracker는 구분되는 두 가지 일을 하며, 각각 다른 방식으로 실패합니다.

#### D-1. Association — "이전 frame과 같은 사람인가?"

매 frame마다 새 detection과 기존 track 사이의 cost를 계산합니다.

```
cost = sqrt( Δx² + Δy² + (radius_residual_weight · Δr)² )
```

`cost < min_correspondence_cost`일 때만 match를 accept합니다. 기존 track은
먼저 앞으로 prediction되므로 mature track의 `Δ`는 전체 변위가 아니라
prediction error입니다. 그러나 **새 track은 velocity가 0**이어서 처음 두
frame에서는 `Δ`가 전체 변위와 같습니다.

| 속도 | 0.1 s frame당 변위 | `0.30` gate와 비교 |
|---|---|---|
| 0.5 m/s | 0.05 m | 충분한 여유 |
| 1.2 m/s | 0.12 m | 문제없음 |
| 1.8 m/s | 0.18 m | radius가 0.1 m 흔들리면 cost 0.21 — 빠듯함 |
| 2.5 m/s | 0.25 m | radius가 조금만 흔들려도 **gate 초과, track이 생성되지 않음** |

| Parameter | 08-12 | 동작 | 올려야 할 때 | 올렸을 때의 비용 |
|---|---|---|---|---|
| `min_correspondence_cost` | `0.3` | 단위 m의 association gate | 빠른 target이 `/tracked_obstacles`로 promote되지 않음(증상 **T**) | 가까운 두 사람이 identity를 바꾸거나 사람이 static object로 "점프"함 |
| `radius_residual_weight` | `0.3` | 거리 대비 radius mismatch에 주는 penalty | frame마다 fitting radius가 흔들리는 사람의 association이 실패함 | 낮추면 크기가 다른 두 물체가 너무 쉽게 associate됨 |
| `std_correspondence_dev` | `0.15` | association distribution 내부에서 가정하는 measurement spread | — | 보통 올바른 parameter가 아니므로 gate를 바꾸십시오 |
| `sensor_rate` | `10.0` | **실제 `/scan` rate와 일치해야 합니다.** Association model과 fade window 크기에 사용 | Stage B를 변경함 | 잘못된 값은 association과 track coasting 시간을 조용히 왜곡함 |
| `tracking_duration` | `1.0` | matching detection이 없을 때 track을 삭제하기 전까지 유지하는 시간(초) | 사람이 간헐적으로 occlusion됨 | 사람이 떠난 뒤에도 ghost circle이 계속 이동함 |

#### D-2. Kalman filter — "정확히 어디에 있고 얼마나 빠른가?"

축별 state는 `[position, velocity]`이며 constant-velocity model입니다.

| Parameter | 08-12 | 역할 |
|---|---|---|
| `measurement_variance` | `1.0` | **R** — 측정된 중심에 가정하는 variance, 단위 **m²**. `1.0`은 LiDAR measurement의 1-σ가 **1 m**라고 주장하는 값입니다 |
| `process_variance` | `0.0001` | **Q(0,0)** — model로 설명되지 않는 position 이동을 얼마나 허용할지 결정 |
| `process_rate_variance` | `0.03` | **Q(1,1)** — step마다 **velocity**가 얼마나 변할 수 있는지, 즉 허용할 acceleration을 결정 |

**`measurement_variance: 1.0`이 화면의 lag를 만드는 이유.** Gain은
`K = P/(P+R)`입니다. YAML comment에 따르면 `R = 1.0`일 때 track의 σ가 약
0.58 m로 수렴하므로 `P ≈ 0.34`이며:

```
K ≈ 0.34 / (0.34 + 1.0) ≈ 0.25
```

Update 한 번에 estimate가 measurement까지 거리의 1/4만 움직이므로 step을
받아들이는 데 약 **0.4초**가 걸립니다. 1.8 m/s로 3 m를 횡단하면 2초도 걸리지
않으므로 **전체 event가 filter transient**입니다. 주황색 원은 파란 점을 계속
쫓아가지만 따라잡지 못하고, velocity 점선은 방향은 맞아도 짧습니다.

파일에도 이 값이 상속받은 잘못된 값이라고 표시되어 있습니다(noiseless sim의
측정 scatter는 1.8e-06 m²였지만 이 역시 shipping value로 쓸 수 없습니다).
실제 Mid-360 + projection + 사람 몸통 circle fit에서는 중심 scatter
**0.1–0.2 m**가 타당한 범위입니다.

```yaml
measurement_variance: 0.04    # sigma = 0.2 m — 보수적인 첫 시도
# measurement_variance: 0.01  # sigma = 0.1 m — 0.04에서도 lag가 남을 때
```

`process_rate_variance: 0.03`은 step당 velocity가 약 0.17 m/s 변할 수 있다는
뜻입니다. 사람은 출발·정지·방향 전환 시 그보다 크게 변합니다. `0.10`(step당
약 0.32 m/s)이 stop-and-go pass를 훨씬 잘 tracking합니다.

> **R과 Q를 함께 tuning할 때:** `measurement_variance`를 낮추거나
> `process_rate_variance`를 높이면 모두 gain이 커집니다. 둘 중 **하나만**
> 바꾸십시오. 지나치면 원이 frame마다 떨리고 velocity 점선의 방향이 요동합니다.
> 그 jitter가 너무 멀리 조정했다는 신호입니다.

#### Stage D 검증

빠른 pass 중 T5에서 다음을 확인합니다.

- **주황색** 원 자체가 존재함(association, D-1)
- 원 중심이 진행 방향 뒤쪽이 아니라 밝은 파란 점 **위에** 있음(KF, D-2)
- 횡단 내내 `uid` label이 **같은 번호**로 유지됨. 한두 frame마다 uid가
  증가하면 track이 다시 생성되는 것이므로 `min_correspondence_cost`로 돌아감
- 점선 velocity line의 길이[m]가 사람 속도[m/s]와 비슷하고 진행 방향을 향함

---

### Stage E — safety gating (`safety_obstacle_filter.yaml`)

이 node는 `/tracked_obstacles`를 controller가 실제 사용할 `/obstacles_safe`로
변환하며 **drop**, **clamp**, **inflate**를 수행합니다.

| Parameter | 08-12 | 동작 | 빠르게 걷는 사람에 대한 설정 |
|---|---|---|---|
| `max_age` | `0.30` s | message stamp가 이 값보다 오래되면 내부의 **모든 원을 폐기**하고 빈 output을 냄 | 10 Hz에서 3 frame을 놓치면 횡단 중 `/obstacles_safe`가 비게 됩니다. Tuning 중에는 **`0.50`** |
| `min_radius` | `0.20` m | 보고하는 radius의 **하한**(drop 아님). 작은 fit도 0.20으로 보고 | 그대로 사용. 몸통 fit은 0.15–0.25 |
| `max_circle_radius` | `0.60` m | 이 radius를 넘으면 **hard drop** | extractor와 **같은 값**을 유지하십시오. Sensing limit은 하나인데 두 파일로 나뉜 것이 과거 bug의 원인이었습니다 |
| `fixed_inflation` | `0.051` m | 모든 radius에 더하는 고정 safety margin(Phase-4 calibration 결과) | 다시 산출하는 경우가 아니면 그대로 둠 |
| `latency_horizon` | `0.12` s | `speed × this`만큼 radius를 추가로 inflate해 pipeline latency를 미리 보상 | 2 m/s에서는 0.24 m가 추가됩니다. Bug가 아니라 올바른 동작입니다 |
| `v_max_obstacle` | `1.5` m/s | Extrapolation 전에 방향은 유지한 채 speed를 **clamp** | 1.8–2.0 m/s 보행자가 clamp되어 빨간 원의 inflation과 velocity read-out이 부족해집니다. 이번 실험은 **`2.5`** |
| `use_covariance` / `k_sigma` | `false` / `2.748` | σ-inflation path. **`false` 유지**. `k_sigma`는 calibration되지 않은 placeholder이며 활성화하려면 `fixed_inflation`과 함께 재calibration해야 함 | 오늘은 변경하지 않음 |

최종 radius는 다음과 같습니다.

```
safe.radius = max(true_radius, min_radius) + fixed_inflation + |v_clamped| · latency_horizon
```

따라서 빨간 원이 주황색 원보다 **큰 것**은 정상이며 속도가 커지면 더 커집니다.

#### Stage E 검증

T5에서 주황색 원이 있는 모든 곳에 **빨간색** 원이 있고, 약간 더 크며, 사람이
빨라질수록 눈에 띄게 커져야 합니다. 주황색은 유지되는데 빨간색이 사라지면
`max_age`에 의해 message 전체가 gating되었거나 `max_circle_radius` 때문에
해당 원이 너무 큰 것입니다.

---

## 8. Decision tree — 어느 stage를 조정할 것인가

§6에서 기록한 문자부터 시작합니다.

```
빠른 pass 중 dpcbf_scan_view를 봅니다.

  사람 위에 파란 점이 없음? ─────────────────▶ N → Stage A (height band)
       │                                            RViz 확인: /livox/lidar에는 보이지만
       │                                            /scan에 없으면 band가 잘못된 것
       ▼
  점이 있지만 5개 미만이거나 두 덩어리? ─────▶ S → Stage A (band를 두껍게, 위로 이동)
       │                                            그다음 C (min_group_points 3)
       ▼
  점은 정상인데 회색 원이 깜빡임? ────────────▶ R → Stage C (grouping / split / merge)
       │                                            r 확인: >0.5면 배경과 merge된 것
       ▼
  회색은 안정적이지만 주황색이 없음? ────────▶ T → Stage D-1 (min_correspondence_cost)
       │                                            uid가 계속 바뀌는지 확인
       ▼
  주황색이 점보다 뒤처짐? ────────────────────▶ L → Stage D-2 (measurement_variance)
       │                                            그다음 process_rate_variance
       ▼
  주황색은 정상인데 빨간색이 없음? ──────────▶ X → Stage E (max_age, max_circle_radius)
       │
       ▼
  세 원이 모두 사람을 따라감 ─────────────────▶ OK. 속도를 높여 반복합니다.
                                                   그다음 margin 확보를 위해 Stage B(20 Hz)를 시도합니다.
```

**당일 적용 규칙:**

1. **Iteration마다 stage 하나만 변경합니다.** Restart에는 30초가 걸리지만 원인이
   섞인 결과를 해석하는 데는 한 시간이 걸립니다.
2. **변경할 때마다 *느린* pass도 다시 수행합니다.** Merge distance, association
   gate, 낮은 `min_group_points`는 false positive를 대가로 빠른 성능을 얻을 수
   있으며, 그 대가가 느린 pass에서 드러납니다.
3. `$SESSION`에 **before/after screenshot**을 저장합니다. `PrtSc`면 충분합니다.
4. 변경은 `install/`에만 두지 말고 **`src/`의 YAML에 기록해 당일 작업이 끝날
   때 commit**합니다.

---

## 9. 복사·붙여넣기용 run sheet

### 9-1. Comp2 / T1 — stack

```bash
source ~/.g1_net_env
cd "$G1_WS"
source /opt/ros/foxy/setup.bash
source install/setup.bash
export SESSION=$G1_WS/evidence/hardware/$(date +%F)/tuning && mkdir -p "$SESSION"

./src/g1_perception/g1_perception_bringup/scripts/g1_hw_preflight.sh || echo "STOP"

# ★ 이 줄 이후 3초 동안 robot을 움직이지 마십시오(DLIO IMU calibration)
ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
    driver:=on lio:=dlio \
    enable_plot_bridge:=true plot_publish_rate:=30.0
```

Comp3 대신 Comp2에서 RViz를 실행하려면 `use_rviz:=true`를 추가합니다.

Live sensor 대신 bag을 replay하는 offline tuning:

```bash
ros2 launch g1_perception_bringup g1_perception_dpcbf.launch.py \
    driver:=off lio:=off
# 다른 terminal에서:
ros2 bag play <bag>
```

### 9-2. Comp2 / T2 — 확인

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/foxy/setup.bash; source install/setup.bash
ros2 daemon stop

# Foxy는 한 번에 topic 하나만 확인합니다. 각 줄 사이에 Ctrl-C를 누르십시오.
ros2 topic hz /livox/lidar          # 10 (Stage B 이후 20)
ros2 topic hz /scan                 # 10   ← Stage A의 성공 여부를 판정
ros2 topic hz /raw_obstacles        # 10
ros2 topic hz /tracked_obstacles    # 10
ros2 topic hz /obstacles_safe       # 10

ros2 run g1_perception_bringup hw_obstacle_watch.py
```

### 9-3. Comp2 / T3 — bag (최소 pass 3과 pass 5 recording)

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/foxy/setup.bash; source install/setup.bash
export SESSION=$G1_WS/evidence/hardware/$(date +%F)/tuning

# Foxy 문법. /livox/lidar를 포함하므로 전체 chain을 offline에서 다시 tuning할 수 있습니다.
ros2 bag record -o "$SESSION/fast_$(date +%H%M%S)" \
    /livox/lidar /livox/imu /odom /tf /tf_static \
    /points_self_filtered /scan /raw_obstacles \
    /tracked_obstacles /obstacles_safe /diagnostics
```

### 9-4. Comp2 / T4 — tuning loop

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/foxy/setup.bash; source install/setup.bash

nano src/g1_perception/g1_perception_bringup/config/pointcloud_to_laserscan.yaml
colcon build --packages-select g1_perception_bringup
ros2 run g1_perception_bringup config_diff.py     # 모두 IDENTICAL
# → 그다음 T1에서 Ctrl-C 후 relaunch
```

### 9-5. Comp3 / T5 — `dpcbf_scan_view`

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/humble/setup.bash; source install/setup.bash

ros2 run dpcbf_plot_client dpcbf_scan_view --ros-args \
    -p range:=4.0 -p scan_history:=5
```

### 9-6. Comp3 / T6 — RViz2

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/humble/setup.bash; source install/setup.bash

rviz2 -d "$G1_WS/src/g1_perception/g1_perception_bringup/rviz/perception_robot_frame.rviz"
```

### 9-7. Comp3 / T7 — odom-frame view (선택 사항)

```bash
source ~/.g1_net_env
cd "$G1_WS"; source /opt/ros/humble/setup.bash; source install/setup.bash

ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py
```

Banner의 `dpcbf/plot: NO DATA (no publisher)`는 **정상**입니다. Robot에는
DPCBF control seam이 없습니다. 왼쪽 panel이 live view이며 오른쪽의 다섯
time series는 비어 있습니다.

### 9-8. 종료 순서

```
① Comp3 T5/T6/T7 — GUI 종료       (Comp2에는 영향 없음)
② Comp2 T3       — bag에서 Ctrl-C  (rosbag finalise가 끝날 때까지 대기)
③ Comp2 T2       — watch에서 Ctrl-C
④ Comp2 T1       — stack에서 Ctrl-C
```

---

## 10. Troubleshooting

| 증상 | 원인 / 해결 방법 |
|---|---|
| **Parameter 변경이 아무 효과가 없음** | ⓐ `ros2 param set`을 사용함 — 여기서는 동작하지 않음(§7.1), ⓑ `colcon build --packages-select g1_perception_bringup`을 하지 않음, ⓒ stack을 재시작하지 않음. `config_diff.py`를 실행했을 때 `DIFFERENT`가 하나라도 나오면 그것이 원인입니다 |
| `/points_self_filtered`는 정상인데 `/scan`이 비어 있음 | Height band가 모든 point를 제외함(Stage A). 또는 `target_frame: base_footprint`를 설정했다면 TF lookup 실패, 즉 DLIO가 `odom → base_link`를 publish하지 않는 상태 |
| `/scan`이 원형 point ring으로 가득함 | `max_height`가 바닥까지 닿았습니다. 값을 낮추십시오(§7-A 한계) |
| Plot에서 모든 것이 좌우 **대칭 반전**됨 | Comp3의 `dpcbf_plot_client`가 오래된 버전입니다. `git pull` 후 rebuild하십시오(§3의 `f9da9c7` 수정). 또는 `/scan`이 `mid360_link`인데 `dpcbf_scan_view`에 `target_frame:=''`를 설정했습니다 |
| 아무것도 없는 곳에 원이 나타남 | Stage C를 지나치게 조정했습니다. `min_group_points`가 너무 낮거나 merge distance 때문에 사람이 벽과 합쳐졌습니다. 느린 pass를 다시 수행하십시오 |
| `uid`가 frame마다 증가함 | Association 실패 → `min_correspondence_cost`(§7-D-1) |
| 주황색 원이 점보다 뒤처짐 | KF gain이 너무 낮음 → `measurement_variance`(§7-D-2) |
| `/tracked_obstacles`는 정상인데 `/obstacles_safe`가 비어 있음 | Safety filter의 `max_age` 또는 `max_circle_radius`(§7-E) |
| RViz가 끊기고 laptop의 `/odom`이 stale 상태가 됨 | `/livox/lidar`가 Wi-Fi bandwidth를 포화시킵니다. Checkbox를 끄거나 Reliability Policy를 Best Effort로 설정하십시오(§5.3) |
| `qt.qpa.plugin: Could not load the Qt platform plugin "xcb"` | SSH session 안에서 GUI를 실행했습니다. **Local** Comp3 terminal에서 실행하십시오(`hostname`으로 확인) |
| Laptop에 topic이 전혀 보이지 않음 | Tuning이 아니라 link 문제입니다 → Fast DDS 매뉴얼 §6-1. 먼저 `ros2 daemon stop` 실행 |
| Restart 후 DLIO가 drift하거나 `/odom`이 튐 | 3초 IMU calibration 중 robot이 움직였습니다. Stack을 재시작하고 가만히 두십시오 |

---

## 11. 기록할 항목

각 iteration마다 session log에 한 줄을 기록합니다.

```
run  stage  changed                          slow  normal  fast  stopgo  toward  note
0    -      baseline (08-12 config)          OK    OK      S     N       R       legs only
1    A      min_height -0.15 / max 0.65      OK    OK      R     R       OK      torso now solid
2    C      min_group_points 3, merge 0.35   OK    OK      T     R       OK      grey stable
3    D      min_corr_cost 0.5, meas_var 0.04 OK    OK      OK    OK      OK      ← keep
```

`$SESSION`에 다음을 저장합니다.

- Pass별 before/after screenshot
- §9-3의 bag
- `ros2 run g1_perception_bringup config_diff.py --json $SESSION/configs.json`
  (bag recording에 실제 사용한 config의 checksum)
- 수치가 필요한 run의
  `hw_obstacle_watch.py -p json:=$SESSION/watch_runN.jsonl`

**당일 작업이 끝나면** `src/`에서 최종 선정한 YAML을 commit하십시오. Comment에
측정한 sensor 높이 `H`와 빠른 보행 속도를 함께 적어 다음 세션에서 어떤 조건에
맞춘 값인지 알 수 있게 합니다.

---

## 관련 문서

- [`g1_fastdds_field_manual_ko.md`](g1_fastdds_field_manual_ko.md) — network
  link, terminal 설정, link troubleshooting(한국어)
- [`dpcbf_plot_visualization.md`](dpcbf_plot_visualization.md) — odom-frame
  plot client
- [`g1_two_computer_setup.md`](g1_two_computer_setup.md) — LiDAR IP,
  `MID360_config.json`, staged bring-up
- [`g1_hardware_preflight.md`](g1_hardware_preflight.md) — 각 preflight 항목의 의미
