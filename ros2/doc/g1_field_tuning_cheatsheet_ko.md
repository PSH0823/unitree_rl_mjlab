# G1 실험 당일 튜닝 치트시트 (2026-08-11)

이 문서는 **노트북 플롯에서 장애물이 잘 안 보이거나, 깜빡이거나, 반경이
이상할 때** 무엇부터 확인하고 어떤 값을 어느 방향으로 바꿀지 빠르게 찾기 위한
현장용 요약이다. 현재 소스의 실제 기본값을 기준으로 작성했다.

> 가장 중요한 원칙: 데이터가 끊긴 문제를 파라미터로 덮지 않는다. 먼저 어느
> 토픽부터 비는지 찾고, 한 번에 한 값만 바꾼 뒤 bag과 변경값을 기록한다.

> **문서보다 현재 source YAML이 우선이다.** 오래된 Stage 설명과 launch 주석
> 일부에는 `min_height: 0.15`와 CycloneDDS 절차가 남아 있다. 현재 실제 값은
> `pointcloud_to_laserscan.yaml`의 `0.70`이고, 오늘 두 컴퓨터 링크는
> `g1_fastdds_field_manual_ko.md`의 Fast DDS 절차를 따른다.

## 0. 오늘의 30초 판정

Computer 2(G1, Foxy)의 점검 pane에서 아래 명령을 **한 줄씩** 실행한다. Foxy의
`ros2 topic hz`에는 토픽을 여러 개 주지 않는다.

```bash
ros2 topic hz /livox/lidar          # 약 10 Hz
ros2 topic hz /odom                 # 약 100 Hz
ros2 topic hz /scan                 # 약 10 Hz
ros2 topic hz /raw_obstacles        # 물체가 없어도 메시지는 약 10 Hz
ros2 topic hz /tracked_obstacles    # 약 10 Hz
ros2 topic hz /obstacles_safe       # 약 10 Hz
```

어디서부터 비는지에 따라 원인이 갈린다.

| 처음 비는 곳 | 먼저 볼 것 | 튜닝 대상 |
|---|---|---|
| `/livox/lidar` | LiDAR IP, host IP, 케이블/전원 | 파라미터 튜닝 문제가 아님 |
| `/odom` | `/livox/imu` 약 200 Hz, DLIO 로그, 기동 후 3초 정지 | 먼저 DLIO 재기동 |
| `/scan` | TF `odom→base_footprint`, 물체의 높이·거리 | height/range band |
| `/raw_obstacles` | `/scan`에 실제 finite range가 있는지 | extractor grouping/radius |
| `/tracked_obstacles` | raw circle은 안정적인지 | tracker association/coast |
| `/obstacles_safe` | safety filter의 stale/large-radius WARN | safety gate |
| C2에서는 정상, C3에서만 없음 | Fast DDS 링크/domain/firewall | perception 파라미터 문제가 아님 |

값이 나오는지 한 프레임만 볼 때:

```bash
ros2 topic echo /scan --once
ros2 topic echo /raw_obstacles --once
ros2 topic echo /tracked_obstacles --once
ros2 topic echo /obstacles_safe --once
```

Computer 3의 GUI에서 이번 실기 기준 정상 배너는 다음이다.

```text
dpcbf/plot: NO DATA       # 정상: 실기 control seam이 아직 없음
odom: ok ...
obstacles_safe: ok ...
```

우측 시계열이 전부 빈 것도 정상이다. `odom` 또는 `obstacles_safe`까지
`NO DATA`/`STALE`일 때만 문제를 추적한다.

## 1. 현재 물체가 보이는 범위

| 항목 | 현재 값 | 설정 파일 |
|---|---:|---|
| 바닥 기준 높이 | 0.70–1.50 m | `config/pointcloud_to_laserscan.yaml` |
| 수평 거리 | 0.30–5.0 m | 같은 파일 |
| 최소 그룹 | 5 scan points | `config/obstacle_detector.yaml` |
| circle fit 상한 | 0.60 m | 같은 파일 |
| 권장 실제 원통 반지름 | 0.52 m 이하 | 거리별 fit bias를 감안한 현장 권장값 |
| self-filter | LiDAR frame에서 x/y ±0.40 m, z −0.55–0.45 m | `config/cropbox_self_filter.yaml` |
| 출력 최소 반지름 | 0.20 m | `config/safety_obstacle_filter.yaml` |

따라서 사람 몸통과 키 큰 원통/상자는 잘 보이지만, 낮은 바닥 물체, 5 m 밖,
0.3 m 안, 넓은 벽/팔레트는 에러 없이 안 보일 수 있다. 벽은 circle 모델에
맞지 않으므로 단순히 상한만 키우는 것으로 신뢰할 수 있는 장애물이 되지 않는다.

## 2. 플롯 자체만 튜닝 (Computer 3)

이 값들은 **검출 결과를 바꾸지 않는다**. 클라이언트를 `Ctrl-C`하고 launch
명령에 인자를 붙여 다시 실행하면 된다.

```bash
cd "$G1_WS"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch dpcbf_plot_client dpcbf_plot_client.launch.py \
    window_s:=60.0 gui_rate_hz:=15.0 stale_after_s:=2.0
```

| 증상/목적 | 조절 방향 | 생기는 변화 |
|---|---|---|
| 노트북이 버벅임 | `gui_rate_hz` 25 → 15 또는 10 | CPU 사용과 화면 갱신 빈도 감소. 데이터 수신률은 그대로 |
| 더 긴 기록을 화면에서 봄 | `window_s` 30 → 60 | 우측 시계열 창이 길어짐. 검출에는 영향 없음 |
| Wi-Fi 순간 흔들림마다 빨간색 | `stale_after_s` 1 → 2 | STALE 판정을 늦춤. 끊긴 데이터가 복구되는 것은 아님 |
| pyqtgraph 창 문제 | `backend:=matplotlib` | 호환성은 좋아질 수 있으나 더 느리고 시계열 수가 줄어듦 |

파일 위치:
`src/g1_perception/dpcbf_plot_client/launch/dpcbf_plot_client.launch.py`.

`plot_publish_rate` 또는 `/dpcbf/plot` 설정은 오늘 perception 화면의 장애물 원과
무관하다. 실기 control seam이 없으므로 그 값을 키워도 우측 그래프는 생기지 않는다.

## 3. 낮거나 먼 물체가 `/scan`에 안 잡힘

수정 파일:
`src/g1_perception/g1_perception_bringup/config/pointcloud_to_laserscan.yaml`.
높이는 `base_footprint`로 변환한 뒤 적용된다.

| 파라미터 (현재값) | 키우면 | 줄이면 |
|---|---|---|
| `min_height: 0.70` | 바닥/다리 return을 더 강하게 제거하지만 낮은 물체를 놓침 | 낮은 물체가 보이지만 바닥 ring과 로봇 다리 오탐이 늘 수 있음 |
| `max_height: 1.50` | 더 높은 부분까지 scan에 포함 | 상부 구조물/몸통 return을 덜 포함하지만 큰 물체를 놓칠 수 있음 |
| `range_min: 0.3` | 근거리 self-hit/노이즈를 더 제거하고 blind zone 증가 | 더 가까운 물체가 보이지만 self-hit 위험 증가 |
| `range_max: 5.0` | 더 먼 점을 허용하지만 희소·깜빡임·오탐 가능성 증가 | 검출 영역이 가까워지고 안정적인 점만 남음 |
| `transform_tolerance: 0.05` | 작은 TF 지연에 더 관대해질 수 있으나 지연을 숨길 수 있음 | TF 타이밍이 빡빡해져 cloud drop 가능성 증가 |

오늘의 보수적인 1회 시험 예시: 낮은 물체만 문제라면 `min_height`를
`0.70 → 0.50` 하나만 바꾸고, 빈 바닥에서 `/scan`의 고정 ring이 생기는지 먼저
본다. ring이 생기면 되돌린다. 로봇 자세가 변할 때만 물체가 사라진다면 단순
height 값보다 extrinsic/자세에 따른 floor overlap 문제일 수 있다.

## 4. `/scan`에는 있는데 `/raw_obstacles`가 안 보이거나 깜빡임

수정 파일:
`src/g1_perception/g1_perception_bringup/config/obstacle_detector.yaml`의
`obstacle_extractor` 블록.

| 파라미터 (현재값) | 키우면 | 줄이면 |
|---|---|---|
| `min_group_points: 5` | 더 많은 점을 요구해 노이즈 감소, 작거나 먼 물체 MISS 증가 | 희소 물체 검출 증가, 점 노이즈/유령 circle 증가 |
| `max_group_distance: 0.10` | 떨어진 점도 같은 그룹으로 묶어 끊긴 물체에 유리, 인접 물체 merge 증가 | 물체가 더 쉽게 분리되지만 한 물체가 조각날 수 있음 |
| `distance_proportion: 0.01745` | 거리가 멀수록 허용 간격을 더 크게 해 원거리 검출 증가, 원거리 merge 증가 | 원거리 그룹이 더 쉽게 끊어짐 |
| `max_split_distance: 0.20` | 굽은/복잡한 그룹을 덜 쪼개 큰 덩어리로 처리 | 선에서 벗어난 점에서 더 잘 분리, 과분할 가능 |
| `max_merge_separation: 0.20` | 가까운 segment를 더 적극적으로 합침, 서로 다른 물체 merge 위험 | 한 물체의 segment가 따로 남을 수 있음 |
| `max_merge_spread: 0.20` | 덜 일직선인 segment도 합침, 잘못된 merge 증가 | merge 조건이 엄격해져 fragmentation 증가 |
| `max_circle_radius: 0.60` | 큰 fitted circle을 통과시키지만 벽/복합 물체를 잘못된 원으로 받을 수 있음 | 큰 원통도 drop됨 |
| `radius_enlargement: 0.17` | extractor가 내보내는 외곽 반경을 키움 | 외곽 반경을 줄임 |

빠른 시험 순서는 다음이 안전하다.

1. 먼 사람/작은 원통이 간헐적으로 사라지면 `min_group_points: 5 → 4`만 시험.
2. 그래도 한 물체가 여러 조각이면 `max_group_distance: 0.10 → 0.12`만 시험.
3. 두 사람이 하나로 합쳐지면 위 값을 다시 줄인다.

`max_circle_radius`를 바꿀 때는
`safety_obstacle_filter.yaml`의 같은 이름도 **반드시 같은 값**으로 바꾼다.
Extractor 상한만 올리면 downstream safety filter가 다시 버린다. 로그의
`dropped a circle...` 또는 `dropped ... true_radius...` WARN도 함께 확인한다.

`radius_enlargement`는 missing detection을 살리는 값이 아니다. 현재 patch에서는
circle-fit 상한 검사 뒤에 더해지며, safety filter는 `true_radius`에서 안전 반경을
다시 만든다. 오늘은 정확한 fixture 측정 없이 건드리지 않는 편이 낫다.

## 5. 원은 잡히는데 ID/속도가 튀거나 잠깐 가려지면 사라짐

수정 파일은 위와 같고 `obstacle_tracker` 블록을 바꾼다.

| 파라미터 (현재값) | 키우면 | 줄이면 |
|---|---|---|
| `tracking_duration: 1.0` | 가림/누락 뒤 트랙을 오래 유지, ghost/coast도 오래 남음 | ghost는 빨리 지워지지만 짧은 가림에도 ID 재생성 |
| `min_correspondence_cost: 0.3` | 더 멀리 움직인 측정도 기존 ID에 연결, 빠른 물체에 유리하나 교차 시 ID swap 증가 | association이 엄격해져 ID 재생성 증가, 가까운 두 물체 혼동 감소 |
| `radius_residual_weight: 0.3` | 반지름 차이를 association에 더 강하게 반영 | 중심 위치 위주로 연결. 크기 측정이 흔들릴 때 유리할 수 있음 |
| `process_rate_variance: 0.03` | 속도 변화에 빨리 반응하지만 화살표가 더 떨릴 수 있음 | 속도는 매끈하지만 가감속 추종이 늦음 |
| `measurement_variance: 1.0` | 측정을 덜 신뢰해 더 매끈하고 느린 추종 | 측정을 더 신뢰해 빠르지만 노이즈를 따라감 |

짧은 가림에만 ID가 끊기면 `tracking_duration: 1.0 → 1.5`를 먼저 시험한다.
빠른 횡단에서 ID가 계속 새로 생기면 `min_correspondence_cost: 0.3 → 0.4`를
시험하되, 두 사람이 교차할 때 ID swap이 늘지 확인한다.

> `measurement_variance: 1.0`은 하드웨어에서 미보정된 값이다. 감으로 바꾸지
> 말고 오늘 bag을 확보한 뒤 `measure_measurement_variance.py`로 산출한다.
> `sensor_rate`, `loop_rate`, `compensate_robot_velocity`, `frame_id`도 오늘의
> 가시성 튜닝값이 아니다. 현재 각각 10 Hz, 10 Hz, false, odom을 유지한다.

## 6. `/tracked_obstacles`는 있는데 `/obstacles_safe`가 작거나 비어 있음

수정 파일:
`src/g1_perception/g1_perception_bringup/config/safety_obstacle_filter.yaml`.

안전 출력 반경은 대략 다음과 같다.

```text
safe radius = max(true_radius, min_radius)
            + fixed_inflation
            + min(speed, v_max_obstacle) * latency_horizon
            + covariance term (현재 OFF)
```

| 파라미터 (현재값) | 키우면 | 주의점 |
|---|---|---|
| `max_age: 0.30` | 늦게 온 tracked 메시지를 덜 버림 | 오래된 장애물을 정상 데이터처럼 허용. 먼저 rate/TF를 고칠 것 |
| `min_radius: 0.20` | 작은 검출도 더 큰 안전 원으로 표시 | 검출 여부·중심 위치는 바뀌지 않음 |
| `max_circle_radius: 0.60` | 큰 true radius 통과 | extractor 값과 반드시 동일하게 유지 |
| `fixed_inflation: 0.051` | 모든 장애물 원이 동일한 양만큼 커짐 | detector MISS나 중심 오차를 숨기는 용도로 즉석 변경 금지 |
| `latency_horizon: 0.12` | 움직이는 물체만 `속도×시간`만큼 더 커짐 | 속도 노이즈도 반경에 반영됨 |
| `v_max_obstacle: 1.5` | 더 빠른 추정 속도까지 보존·팽창 | 비정상 velocity spike의 영향도 커짐 |

`use_covariance`는 **false 유지**한다. 현재 `k_sigma: 2.748`은 이 branch에서
하드웨어 보정된 값이 아니며, 잘못된 `measurement_variance`와 함께 켜면 원이
근거 없이 크게 부풀 수 있다.

`fixed_inflation`, `min_radius`, `latency_horizon`을 키우면 이미 검출된 원만
커진다. 화면에 아예 없는 장애물을 되살리지 못한다.

## 7. 로봇 자기 몸이 장애물로 따라다님

수정 파일:
`src/g1_perception/g1_perception_bringup/config/cropbox_self_filter.yaml`.
이 박스는 `base_footprint`가 아니라 **LiDAR cloud frame (`mid360_link`)**이다.
`negative: true`이므로 박스 안의 점을 삭제한다.

| 변경 | 효과 | 대가 |
|---|---|---|
| x/y min의 절댓값과 max를 키움 | 팔/몸통 self-hit를 더 많이 제거 | 로봇 근처 실제 물체도 사라지는 blind zone 증가 |
| `min_z`를 더 낮춤 | LiDAR 아래 self-hit를 더 제거 | 아래쪽 실제 물체 손실 |
| `max_z`를 높임 | LiDAR 위쪽 self-hit를 더 제거 | 위쪽 실제 물체 손실 |

박스를 바로 키우기 전에 self-hit bag을 남긴다. 팔이 `range_min: 0.30` 부근까지
나온다면 큰 직육면체 하나로 해결할 문제가 아니라 shaped/pose-aware mask가
필요하다. 빈 화면이 깨끗하다는 이유만으로 성공으로 판단하지 않는다.

## 8. `/odom` 또는 TF가 불안정함

설정 파일:
`src/g1_perception/g1_perception_bringup/config/dlio.yaml`.

오늘 첫 조치는 튜닝이 아니라 스택을 재기동하고 **첫 3초 동안 로봇을 완전히
정지**시키는 것이다. `/livox/imu`가 약 200 Hz인지도 먼저 확인한다.

| 파라미터 | 키우면/줄이면 | 현장 판단 |
|---|---|---|
| `odom/preprocessing/cropBoxFilter/size: 1.0` | 키우면 LiDAR 주변(지면 포함)을 더 제거, 줄이면 근거리 geometry를 더 사용 | self-hit bag 없이 변경하지 않음 |
| `odom/preprocessing/voxelFilter/res: 0.25` | 키우면 CPU↓/세부정보↓, 줄이면 CPU↑/점 밀도↑ | CPU가 실제 병목일 때만 |
| `odom/imu/calibration/time: 3.0` | 키우면 초기 bias 추정 시간이 늘어남 | 시간보다 “그 동안 완전 정지”가 우선 |

`extrinsics/baselink2*`, `frames/*`, `use_sim_time`은 현장 튜닝 금지다. Extrinsic은
`g1_description/urdf/g1_mid360.xacro`가 단일 원본이며 YAML을 직접 고치면 guard가
깨지거나 센서 변환이 이중 적용될 수 있다.

CPU가 부족할 때만 launch에 `voxel:=on`을 붙일 수 있다. 이 경우 0.05 m
VoxelGrid가 projection 앞에 추가되어 CPU는 줄 수 있지만, 희소/원거리 물체의
점 수가 줄어 detector MISS가 늘 수 있으므로 `/scan`과 `/raw_obstacles`를 비교한다.

## 9. 설정 변경 적용법 (Computer 2)

Perception YAML은 설치 artefact다. **소스 YAML만 고치고 launch하면 예전 값으로
실행된다.** 또한 현재 노드들은 내부 계산값을 기동 시 읽으므로 `ros2 param set`
만으로 현장 튜닝하지 않는다. YAML 수정 → bringup 재설치 → 스택 재기동 순서다.

변경 전 세션 폴더에 기준값을 남긴다.

```bash
cd "$G1_WS"
: "${SESSION:?SESSION을 먼저 안전한 세션 디렉터리로 설정하십시오}"
mkdir -p "$SESSION/configs_before_tuning"
cp src/g1_perception/g1_perception_bringup/config/*.yaml \
   "$SESSION/configs_before_tuning/"
```

YAML 하나에서 값 하나만 수정한 뒤:

```bash
cd "$G1_WS"
source /opt/ros/foxy/setup.bash
colcon build --packages-select g1_perception_bringup
source install/setup.bash
python3 src/g1_perception/g1_perception_bringup/scripts/config_diff.py
# config_diff: PASS 확인 후 기존 스택 Ctrl-C, 다시 launch
```

재기동 뒤 실제 로드값과 변경 기록을 남긴다.

```bash
ros2 param get /pointcloud_to_laserscan min_height
ros2 param get /obstacle_extractor min_group_points
ros2 param get /obstacle_tracker tracking_duration
ros2 param get /safety_obstacle_filter fixed_inflation

git diff -- src/g1_perception/g1_perception_bringup/config \
  | tee "$SESSION/tuning_changes.diff"
```

플롯 인자만 바꿀 때는 C3 클라이언트만 재기동하면 되며 C2 재빌드는 필요 없다.

## 10. 오늘 권장하는 증상별 최소 변경

| 증상 | 첫 변경 | 반드시 같이 확인 |
|---|---|---|
| 낮은 물체가 안 보임 | `min_height` 0.70 → 0.50 시험 | 빈 바닥 ring, 로봇 다리 phantom |
| 먼 물체가 간헐적으로 MISS | `min_group_points` 5 → 4 시험 | false circle, `/scan` 자체의 점 유무 |
| 한 물체가 여러 조각 | `max_group_distance` 0.10 → 0.12 시험 | 두 물체가 하나로 merge되는지 |
| 0.3–1초 가림 뒤 ID 재생성 | `tracking_duration` 1.0 → 1.5 시험 | 사라진 ghost가 오래 남는지 |
| 빠른 횡단에서 ID 재생성 | `min_correspondence_cost` 0.3 → 0.4 시험 | 교차 시 ID swap |
| 노트북 화면만 버벅임 | C3 `gui_rate_hz` 25 → 15 | C2 topic rate는 그대로인지 |
| Wi-Fi 순간 흔들림 표시만 완화 | C3 `stale_after_s` 1 → 2 | 실제 데이터 loss는 별도 해결 |
| self phantom | 바로 박스를 키우지 말고 raw self-hit bag | 실제 0.4/0.6/0.8 m 물체가 남는지 |
| 큰 원통 drop WARN | 두 파일의 `max_circle_radius`를 함께 검토 | 벽/복합체 circle 모델 오용 금지 |

위 숫자는 **한 단계 원인 분리용 시험값**이지 새 기본값이 아니다. 좋아 보인다는
이유만으로 commit하지 말고, baseline과 변경 후 bag을 같은 장면에서 각각 남긴다.

## 11. 원문/코드 위치

- 오늘의 전체 실행 및 Fast DDS 문제 해결:
  `doc/g1_fastdds_field_manual_ko.md` (§5–§6)
- 두 컴퓨터 실행, 물체 가시 범위, 증상표:
  `doc/g1_two_computer_setup.md` (D-4, E-1–E-3)
- 단계별 측정법과 중단 기준:
  `doc/g1_first_perception_experiment.md` (Stage 7–12)
- 플롯 인자와 화면 의미:
  `doc/dpcbf_plot_visualization.md`
- 파라미터 실제 로딩 구성:
  `src/g1_perception/g1_perception_bringup/launch/perception.launch.py`
- 플롯 launch 인자:
  `src/g1_perception/dpcbf_plot_client/launch/dpcbf_plot_client.launch.py`
- extractor/tracker 파라미터 원뜻:
  `src/external/obstacle_detector_2/README.md`
