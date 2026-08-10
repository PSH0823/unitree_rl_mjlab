# 장애물 인지·추정 파이프라인 (한국어 정리)

이 문서는 G1 로봇의 **LiDAR 원시 점군 → 장애물 원(circle) 추정 → DPCBF 안전 필터**까지의
전체 경로를, 실제 코드와 함께 단계별로 정리한 것이다.

- 대상 브랜치: `obstacle_detection`
- 관련 패키지: [ros2/src/g1_perception/](../ros2/src/g1_perception/), [ros2/src/external/obstacle_detector_2/](../ros2/src/external/obstacle_detector_2/), [dpcbf/](../dpcbf/), [simulate/src/main.cc](../simulate/src/main.cc)

---

## 0. 한눈에 보는 전체 흐름

```
 [센서]
   HW : livox_ros_driver2  ─┐
   SIM: sim_mjlidar_bridge ─┴──▶ /livox/lidar   (sensor_msgs/PointCloud2)
                                     │
                                     ▼
 [1] pcl_ros::CropBox                      자기 몸체 반사 제거
                                ──▶ /points_self_filtered
                                     │
                                     ▼
 [2] pointcloud_to_laserscan              3D 점군 → 2D 스캔 투영
                                ──▶ /scan          (sensor_msgs/LaserScan)
                                     │
                                     ▼
 [3] obstacle_extractor                   점 그룹화 → 선분 → 원 피팅
                                ──▶ /raw_obstacles  (obstacle_detector/Obstacles)
                                     │
                                     ▼
 [4] obstacle_tracker                     데이터 연관 + 칼만필터(위치/속도/반경)
                                ──▶ /tracked_obstacles
                                     │
                                     ▼
 [5] safety_obstacle_filter               게이팅 + 안전 반경 팽창
                                ──▶ /obstacles_safe
                                     │
                                     ▼
 [6] dpcbf_ros_adapter :: ObstacleSource  구독 → 무잠금 버퍼 → 외삽/staleness
                                     │
                                     ▼
 [7] simulate/src/main.cc (1 kHz 제어루프) ──▶ dpcbf::DpcbfSafetyFilter::Filter()
```

배선(remapping)은 전부 한 파일에서 정의된다 →
[perception.launch.py:30-91](../ros2/src/g1_perception/g1_perception_bringup/launch/perception.launch.py#L30-L91)

**중요**: [1]~[5]는 ROS2 컴포저블 노드로 **하나의 컨테이너(`perception_container`) 안에서
intra-process 통신**으로 돌아간다. 즉 노드 간 직렬화/DDS 비용이 없다.
[6]만 DDS를 실제로 타고 제어 프로세스로 넘어온다.

---

## 1. 노드 / 토픽 요약표

| # | 노드 | 패키지 | 입력 토픽 | 출력 토픽 | 소스 |
|---|------|--------|-----------|-----------|------|
| 0 | `livox_lidar_publisher` (HW) | `livox_ros_driver2` | — | `/livox/lidar`, `/livox/imu` | [source_hw.launch.py](../ros2/src/g1_perception/g1_perception_bringup/launch/source_hw.launch.py) |
| 0 | `sim_mjlidar_bridge` (SIM) | `sim_mjlidar_bridge` | MuJoCo 내부 | `/livox/lidar` | [source_sim.launch.py](../ros2/src/g1_perception/g1_perception_bringup/launch/source_sim.launch.py) |
| 1 | `crop_box_self_filter` | `pcl_ros` | `/livox/lidar` | `/points_self_filtered` | (외부) |
| 2 | `pointcloud_to_laserscan` | `pointcloud_to_laserscan` | `/points_self_filtered` | `/scan` | (외부) |
| 3 | `obstacle_extractor` | `obstacle_detector` | `/scan` | `/raw_obstacles` | [obstacle_extractor.cpp](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp) |
| 4 | `obstacle_tracker` | `obstacle_detector` | `/raw_obstacles`, `/odom` | `/tracked_obstacles` | [obstacle_tracker.cpp](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp) |
| 5 | `safety_obstacle_filter` | `safety_obstacle_filter` | `/tracked_obstacles` | `/obstacles_safe` | [safety_obstacle_filter_node.cpp](../ros2/src/g1_perception/safety_obstacle_filter/src/safety_obstacle_filter_node.cpp) |
| 6 | `dpcbf_ros_adapter` | `dpcbf_ros_adapter` | `/obstacles_safe` | `/dpcbf/status`, `/dpcbf/plot` | [obstacle_source.cpp](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp) |

---

## 2. 【핵심 답변】 장애물 위치를 받아오는 코드

제어 측에서 장애물을 **최종적으로 받아오는 지점**은 `dpcbf_ros_adapter`의 `ObstacleSource`다.

### 2.1 퍼블리셔 (보내주는 쪽)

[safety_obstacle_filter_node.cpp:34-35](../ros2/src/g1_perception/safety_obstacle_filter/src/safety_obstacle_filter_node.cpp#L34-L35)

```cpp
pub_ = create_publisher<obstacle_detector::msg::Obstacles>(
    "obstacles_safe", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
```

- 변수명: **`pub_`**
- 토픽: **`/obstacles_safe`** (Reliable, depth 1 — "최신 것이 이긴다")

### 2.2 섭스크라이버 (받아오는 쪽)

[obstacle_source.cpp:298-311](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L298-L311)

```cpp
if (cfg.mode != Mode::kOracle) {
  // /obstacles_safe is Reliable depth 1 — latest wins (§7.1).
  impl_->sub =
      impl_->node->create_subscription<obstacle_detector::msg::Obstacles>(
          cfg.topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
          // ConstSharedPtr, not `const Obstacles&`: rclcpp accepts
          // const-reference subscription callbacks only from Galactic on,
          // and the G1's onboard computer runs Foxy.
          [impl = impl_.get()](
              obstacle_detector::msg::Obstacles::ConstSharedPtr m) {
            impl->OnObstacles(*m);
          });
}
```

- 변수명: **`impl_->sub`** (타입 `rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr`, [obstacle_source.cpp:75](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L75))
- 토픽: `cfg.topic` — [dpcbf_ros_adapter.yaml:13](../ros2/src/g1_perception/g1_perception_bringup/config/dpcbf_ros_adapter.yaml#L13)에서 `/obstacles_safe`로 로드
- `ConstSharedPtr` 콜백을 쓰는 이유: G1 온보드가 **ROS 2 Foxy**라서 const-reference 콜백을 못 받음

### 2.3 수신 콜백 — 여기서 변수에 담긴다

[obstacle_source.cpp:82-102](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L82-L102)

```cpp
void OnObstacles(const obstacle_detector::msg::Obstacles& msg) {
  ObstacleFrame frame;
  frame.stamp = StampToSec(msg.header.stamp);
  std::size_t n = 0;
  for (const auto& c : msg.circles) {
    if (n >= kMaxObstacles) {
      dropped_circles.fetch_add(msg.circles.size() - n,
                                std::memory_order_relaxed);
      break;
    }
    auto& o = frame.obstacles[n++];
    o.x          = c.center.x;      // ← 장애물 x 위치
    o.y          = c.center.y;      // ← 장애물 y 위치
    o.radius     = c.radius;        // ← 안전 반경(팽창 포함)
    o.velocity_x = c.velocity.x;    // ← 장애물 x 속도
    o.velocity_y = c.velocity.y;    // ← 장애물 y 속도
    o.id         = UidToId(c.uid);  // ← 트랙 ID
  }
  frame.count = n;
  buffer.Publish(frame);            // 무잠금 더블버퍼에 게시
}
```

**받아온 변수 이름 정리**

| 의미 | ROS 메시지 필드 | 내부 변수 |
|---|---|---|
| 프레임 타임스탬프 | `msg.header.stamp` | `frame.stamp` (초 단위 double) |
| 장애물 개수 | `msg.circles.size()` | `frame.count` |
| 위치 x, y | `c.center.x`, `c.center.y` | `o.x`, `o.y` |
| 속도 | `c.velocity.x/.y` | `o.velocity_x`, `o.velocity_y` |
| 반경 | `c.radius` | `o.radius` |
| ID | `c.uid` (uint64) | `o.id` (int, 하위 31비트) |

`kMaxObstacles = 128` ([obstacle_buffer.h:23](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L23)).
Phase 3 실측 최대 동시 트랙은 22개였으므로 여유 있는 하드캡이다. 초과분은 `dropped_circles`로 카운트된다.

### 2.4 제어 루프에서 꺼내 쓰는 곳

[main.cc:1008-1018](../simulate/src/main.cc#L1008-L1018)

```cpp
JoystickAxisFilter axis_filter = [dpcbf_body_id, seam_limits](
                                     float lx, float ly, float rx) {
  const dpcbf::RobotState robot = ReadRobotGroundTruth(m, d, dpcbf_body_id);
  obstacle_source->SetRobotXY(robot.x, robot.y);
  const double t_query = d->time;  // sim time, same clock as header stamps
  auto snap = obstacle_source->GetObstacles(t_query);   // ★ 여기서 받아옴
  const auto desired = dra::AxesToDesired(lx, ly, rx, seam_limits);
  // §10.3 degrade ramp applied at the call site, before Filter() (§10.5).
  const auto scaled = dra::ScaleDesired(desired, snap.command_scale);
  const auto filtered = safety_filter.Filter(robot, scaled, snap.obstacles);
  ...
```

- **`snap.obstacles`** → 타입 `std::vector<dpcbf::ObstacleState>`
- **`snap.command_scale`** → staleness에 따른 명령 감쇠 계수 (1.0 → 0.0)
- **`snap.age_s`**, **`snap.state`** → 데이터 나이와 상태(fresh/degrade/stop/no_data)

### 2.5 최종 자료형

[dpcbf_safety_filter.h:19-26](../dpcbf/include/dpcbf/dpcbf_safety_filter.h#L19-L26)

```cpp
struct ObstacleState {
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
  int id = -1;
};
```

### 2.6 ⚠️ 모드에 따라 섭스크라이버가 안 생길 수 있다

[main.cc:909-924](../simulate/src/main.cc#L909-L924)

```cpp
// Mode selection (launch-selectable via env; DEFAULT REMAINS ORACLE, D5).
auto mode = dra::ObstacleSource::Mode::kOracle;
if (const char* mode_env = std::getenv("UNITREE_DPCBF_MODE")) {
  const std::string mode_str = mode_env;
  if (mode_str == "oracle")         mode = dra::ObstacleSource::Mode::kOracle;
  else if (mode_str == "shadow")    mode = dra::ObstacleSource::Mode::kShadow;
  else if (mode_str == "estimated") mode = dra::ObstacleSource::Mode::kEstimated;
  ...
```

| 모드 | 환경변수 | 동작 |
|---|---|---|
| `oracle` (**기본값**) | `UNITREE_DPCBF_MODE=oracle` | 토픽 구독 안 함. 시뮬레이터 정답값(`oracle_provider`)을 그대로 사용 |
| `shadow` | `=shadow` | 제어는 정답값으로 하되, 추정값을 함께 받아 **오차 통계**만 계산 |
| `estimated` | `=estimated` | 실제 추정 장애물(`/obstacles_safe`)로 제어 |

정답값 제공 람다 → [main.cc:895-907](../simulate/src/main.cc#L895-L907)

```cpp
const auto oracle_provider = [] {
  std::vector<dpcbf::ObstacleState> obstacle_states;
  const auto obstacle_snapshot = dynamic_obstacles.Snapshot();
  obstacle_states.reserve(obstacle_snapshot.size());
  for (std::size_t obstacle_id = 0; obstacle_id < obstacle_snapshot.size();
       ++obstacle_id) {
    const auto& obstacle = obstacle_snapshot[obstacle_id];
    obstacle_states.push_back({obstacle.position[0], obstacle.position[1],
                               obstacle.radius,
                               obstacle.velocity[0], obstacle.velocity[1],
                               static_cast<int>(obstacle_id)});
  }
  return obstacle_states;
};
```

---

## 3. 【핵심 답변】 장애물 추정 과정의 핵심 코드

### 3.1 [1단계] 자기 몸체 제거 — CropBox

코드는 외부 패키지(`pcl_ros::CropBox`)라 이 저장소엔 파라미터만 있다.

[cropbox_self_filter.yaml](../ros2/src/g1_perception/g1_perception_bringup/config/cropbox_self_filter.yaml)

```yaml
crop_box_self_filter:
  ros__parameters:
    min_x: -0.40
    max_x:  0.40
    min_y: -0.40
    max_y:  0.40
    min_z: -0.55
    max_z:  0.45
    negative: true          # 박스 "안쪽"을 제거 (= 로봇 몸체 반사)
    input_frame: ''         # 클라우드 프레임(mid360_link) 기준 — TF 미개입
```

> 박스가 Appendix A(±0.35 / max_z 0.25)보다 커진 이유: 서 있는 자세에서 **손목 링크 반사**가
> 0.29–0.36 m 거리에 잡혀, 두 클러스터가 병합되면서 `true_radius ≈ 0.41`짜리 **유령 원**이
> p_max 안쪽 0.29 m에 생겼기 때문이다.

### 3.2 [2단계] 3D → 2D 투영

[pointcloud_to_laserscan.yaml](../ros2/src/g1_perception/g1_perception_bringup/config/pointcloud_to_laserscan.yaml)

```yaml
pointcloud_to_laserscan:
  ros__parameters:
    target_frame: base_footprint
    min_height: 0.70        # ← 바닥 반사를 걸러내는 유일한 수단
    max_height: 1.50
    angle_min: -3.14159265
    angle_max:  3.14159265
    angle_increment: 0.0058 # ~0.33 deg, ~1080 bins
    range_min: 0.3
    range_max: 5.0
    scan_time: 0.1          # 10 Hz
```

> **주의**: 이 워크스페이스에 **지면 분할(ground segmentation)은 없다.**
> `ground_seg:=patchwork` 인자는 no-op이었고 지금은 명시적 에러로 막혀 있다
> ([bringup.launch.py:23-40](../ros2/src/g1_perception/g1_perception_bringup/launch/bringup.launch.py#L23-L40)).
> 바닥을 거르는 건 오직 위의 `min_height`뿐이다.

### 3.3 [3단계] 검출 — 원(circle) 추출 ★

파일: [obstacle_extractor.cpp](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp)

#### (a) 입력 콜백

[obstacle_extractor.cpp:160-174](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L160-L174)

```cpp
void ObstacleExtractor::scanCallback(const sensor_msgs::msg::LaserScan& scan_msg) {
  base_frame_id_ = scan_msg.header.frame_id;
  stamp_ = scan_msg.header.stamp;

  double phi = scan_msg.angle_min;

  for (const float r : scan_msg.ranges) {
    if (r >= scan_msg.range_min && r <= scan_msg.range_max)
      input_points_.push_back(Point::fromPoolarCoords(r, phi));

    phi += scan_msg.angle_increment;
  }

  processPoints();
}
```

#### (b) 메인 파이프라인

[obstacle_extractor.cpp:219-234](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L219-L234)

```cpp
void ObstacleExtractor::processPoints() {
  segments_.clear();
  circles_.clear();

  groupPoints();  // Grouping points simultaneously detects segments
  mergeSegments();

  detectCircles();
  mergeCircles();

  transformObstacles();
  publishObstacles();
  publishVisualizationObstacles();

  input_points_.clear();
}
```

#### (c) 점 그룹화 — 거리 비례 임계값

[obstacle_extractor.cpp:236-285](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L236-L285)

```cpp
for (PointIterator point = std::next(input_points_.begin()); point != input_points_.end(); ++point) {
  double range = (*point).getRange();
  double distance = (*point - *point_set.end).length();

  if (distance < p_max_group_distance_ + range * p_distance_proportion_) {
    point_set.end = point;
    point_set.num_points++;
  }
  else {
    double prev_range = (*point_set.end).getRange();

    // Heron's equation
    double p = (range + prev_range + distance) / 2.0;
    double S = sqrt(p * (p - range) * (p - prev_range) * (p - distance));
    double sin_d = 2.0 * S / (range * prev_range); // Sine of angle between beams

    if (abs(sin_d) < sin_dp && range < prev_range)
      point_set.is_visible = false;

    detectSegments(point_set);
    ...
```

- 그룹 임계값 = `max_group_distance + range × distance_proportion`
  (멀수록 빔 간격이 벌어지므로 거리 비례 항 추가)
- **포크 패치**: 루프가 `std::next(begin())`부터 시작한다. 업스트림은 `begin()++`(post-increment)
  때문에 첫 점을 두 번 세어 첫 그룹의 `num_points`가 1 부풀고, `fitSegment`가 그룹 밖 점 하나를
  읽어 선분 피팅이 오염됐다.

#### (d) 선분 → 원 피팅 ★

[obstacle_extractor.cpp:397-444](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L397-L444)

```cpp
void ObstacleExtractor::detectCircles() {
  for (auto segment = segments_.begin(); segment != segments_.end(); ++segment) {
    if (p_circles_from_visibles_) {
      bool segment_is_visible = true;
      for (const PointSet& ps : segment->point_sets) {
        if (!ps.is_visible) { segment_is_visible = false; break; }
      }
      if (!segment_is_visible) continue;
    }

    Circle circle(*segment);

    // P-4: gate on the FITTED radius, not on the fitted radius plus the
    // safety enlargement.
    if (circle.radius < p_max_circle_radius_) {
      circle.radius += p_radius_enlargement_;
      circles_.push_back(circle);

      if (p_discard_converted_segments_) {
        segment = segments_.erase(segment);
        --segment;
      }
    } else {
      // A dropped obstacle is invisible to everything downstream ...
      ++dropped_large_circles_;
      RCLCPP_WARN_THROTTLE(
          nh_->get_logger(), *nh_->get_clock(), 2000,
          "obstacle_extractor: dropped a circle of fitted radius %.3f m "
          "(max_circle_radius %.3f m) — an obstacle this large is INVISIBLE "
          "downstream; %ld dropped since start",
          circle.radius, p_max_circle_radius_, dropped_large_circles_);
    }
  }
}
```

> **포크 패치 P-4 (gap G2)**: 업스트림은 `radius_enlargement`를 **먼저 더한 뒤**
> `max_circle_radius`와 비교했다. 그래서 파라미터의 실제 의미가
> "최대 반경 − radius_enlargement"였고, 팽창을 보수적으로 키울수록 **센서가 볼 수 있는
> 최대 장애물이 조용히 작아졌다**. 게다가 부분 가림이 생기면 피팅 반경이 줄어 다시 통과하므로
> **간헐적으로** 장애물이 사라졌다. 지금은 피팅 반경으로 게이트하고, 드롭될 때마다 카운트 + 경고한다.

원 피팅 수식 자체:
[utilities/figure_fitting.h](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/figure_fitting.h),
[utilities/circle.h](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/circle.h)

#### (e) 좌표 변환 (base → odom)

[obstacle_extractor.cpp:586-620](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L586-L620)

```cpp
try {
    // Look up at the scan stamp (not latest): output must be consistent
    // with the pose at measurement time, and replay must be deterministic.
    m_transform = tf_buffer_->lookupTransform(p_frame_id_, base_frame_id_, stamp_,
                                              rclcpp::Duration::from_seconds(0.1));
}
```

- `stamp_`(측정 시각)로 TF를 조회한다. `latest`가 아니라는 점이 중요 —
  재생(replay) 결정성과 측정 시점 자세 일관성을 위해서.
- 출력 프레임은 `frame_id: odom` (yaml).

### 3.4 [4단계] 추적 — 데이터 연관 + 칼만필터 ★★

파일: [obstacle_tracker.cpp](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp)

#### (a) 두 가지 사이클 모드

[obstacle_tracker.cpp:222-266](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L222-L266)

```cpp
void ObstacleTracker::obstaclesCallback(const obstacle_detector::msg::Obstacles::ConstSharedPtr& new_obstacles) {
  if (timerMode()) {
    // A-mode: the timer owns predict/prune/publish; the callback only
    // associates and stores the fresh measurement via correctState().
    obstaclesCallbackCircles(new_obstacles);
    obstaclesCallbackSegments(new_obstacles);
    return;
  }

  // P-2 measurement-driven cycle: dt from header stamps, predict-only for
  // every live track, then associate + correct, then prune faded tracks and
  // publish with the measurement stamp (deterministic under replay).
  rclcpp::Time stamp(new_obstacles->header.stamp);
  double dt = p_sampling_time_;
  if (have_measurement_) {
    dt = (stamp - last_measurement_stamp_).seconds();
    if (dt <= 0.0 || dt > p_tracking_duration_)
      dt = p_sampling_time_;  // clock jump / replay restart: fall back to nominal
  }
  last_measurement_stamp_ = stamp;
  have_measurement_ = true;

  m_last_dt_ = dt;
  TrackedCircleObstacle::setSamplingTime(dt);
  ...
  for (auto& t : tracked_circle_obstacles_)
    t.predictState();
  ...
  obstaclesCallbackCircles(new_obstacles);
  ...
  publishObstacles();
```

| 모드 | 트리거 | 특징 |
|---|---|---|
| **A-mode** (업스트림, `loop_rate` 타이머) | [timerCallback:197](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L197) | 고정 주기로 predict+correct. 측정 없어도 이전 `y` 재사용 |
| **P-2 측정 구동** (포크 기본) | 위 콜백 | 헤더 스탬프에서 `dt` 계산, predict-only 후 연관/보정. **재생 결정적** |

#### (b) 데이터 연관 — 비용 행렬

[obstacle_tracker.cpp:505-533](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L505-L533)

```cpp
double ObstacleTracker::obstacleCostFunction(const CircleObstacle& new_obstacle,
                                             const CircleObstacle& old_obstacle) {
  ...
  // P-2: radius residual down-weighted (Appendix-A weight 0.3) — the radius
  // estimate is far noisier than the center estimate and was causing missed
  // associations at full weight.
  cost = sqrt(pow(new_obstacle.center.x - old_obstacle.center.x, 2.0)
            + pow(new_obstacle.center.y - old_obstacle.center.y, 2.0)
            + pow(p_radius_residual_weight_ * (new_obstacle.radius - old_obstacle.radius), 2.0));
  ...
  return cost / 1.0;
}
```

[obstacle_tracker.cpp:541-561](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L541-L561)

```cpp
/*
 * Cost between two obstacles represents their difference.
 * The bigger the cost, the less similar they are.
 * N rows of cost_matrix represent new obstacles.
 * T+U columns of cost matrix represent old tracked and untracked obstacles.
 */
cost_matrix = mat(N, T + U, fill::zeros);

for (int n = 0; n < N; ++n) {
  for (int t = 0; t < T; ++t)
    cost_matrix(n, t) = obstacleCostFunction(new_obstacles[n], tracked_circle_obstacles_[t].getObstacle());

  for (int u = 0; u < U; ++u)
    cost_matrix(n, u + T) = obstacleCostFunction(new_obstacles[n], untracked_circle_obstacles_[u]);
}
```

- 행 = 새 측정 N개, 열 = 기존 **추적 중 트랙 T개 + 미확정(untracked) U개**
- 매칭 게이트: `min_correspondence_cost: 0.3` (= 0.30 m)

#### (c) 연관 본체 — 융합/분열/신규

[obstacle_tracker.cpp:268-388](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L268-L388)

핵심 흐름:

1. **fusion(융합)** — 여러 기존 트랙이 하나의 새 측정으로 합쳐짐 → `fuseObstacles()`
2. **fission(분열)** — 하나의 기존 트랙이 여러 새 측정으로 갈라짐 → `fissureObstacle()`
3. **1:1 매칭** → `correctState()` (칼만 보정)
4. **신규 트랙 승격** (untracked → tracked)
5. **매칭 실패** → `untracked_circle_obstacles_`에 보관 (다음 프레임에 승격 기회)

신규 트랙 승격 부분이 특히 중요하다:

[obstacle_tracker.cpp:349-370](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L349-L370)

```cpp
else if (row_min_indices[n] >= T) {
  // P-2 two-point track initiation: promotion has two measurements in
  // hand (last frame's untracked one and this frame's), so seed the KF
  // velocity from their difference instead of zero — initKF copies
  // obstacle_.velocity into the state. A zero-velocity start makes the
  // filter lag a fast crosser for many frames after confirmation.
  obstacle_detector::msg::CircleObstacle seed = untracked_circle_obstacles_[row_min_indices[n] - T];
  if (m_last_dt_ > 0.0) {
    seed.velocity.x = (new_obstacles->circles[n].center.x - seed.center.x) / m_last_dt_;
    seed.velocity.y = (new_obstacles->circles[n].center.y - seed.center.y) / m_last_dt_;
  }
  TrackedCircleObstacle to(seed);
  to.correctState(new_obstacles->circles[n]);
  if (timerMode())  // upstream catch-up: settle the fresh track onto the timer phase
    for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
      to.updateState();
  new_tracked_obstacles.push_back(to);
}
```

> **포크 패치 P-2 (2점 초기화)**: 업스트림은 속도 0으로 시작해서, 빠르게 가로지르는 장애물을
> 확정 후에도 여러 프레임 동안 놓쳤다. 이제 두 측정의 차분으로 속도를 시드한다.

#### (d) 칼만필터 본체 ★★

트랙 하나당 **x / y / r 축을 각각 독립 KF**로 돌린다 (상태 2차원: [값, 변화율]).

[tracked_circle_obstacle.h:50-53](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L50-L53)

```cpp
TrackedCircleObstacle(const obstacle_detector::msg::CircleObstacle& obstacle)
  : obstacle_(obstacle), kf_x_(0, 1, 2), kf_y_(0, 1, 2), kf_r_(0, 1, 2) {
  fade_counter_ = s_fade_counter_size_;
  setNewUid();
  initKF();
}
```

`KalmanFilter(dim_in=0, dim_out=1, dim_state=2)` — 입력 없음, 관측 1개, 상태 2개.

**예측 (predict)** — [tracked_circle_obstacle.h:56-71](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L56-L71)

```cpp
void predictState() {
  applyTimestep();
  kf_x_.predictState();
  kf_y_.predictState();
  kf_r_.predictState();

  obstacle_.center.x = kf_x_.q_pred(0);
  obstacle_.center.y = kf_y_.q_pred(0);

  obstacle_.velocity.x = kf_x_.q_pred(1);
  obstacle_.velocity.y = kf_y_.q_pred(1);

  obstacle_.radius = kf_r_.q_pred(0);

  fade_counter_--;
}
```

**보정 (correct)** — [tracked_circle_obstacle.h:73-91](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L73-L91)

```cpp
void correctState(const obstacle_detector::msg::CircleObstacle& new_obstacle) {
  kf_x_.y(0) = new_obstacle.center.x;
  kf_y_.y(0) = new_obstacle.center.y;
  kf_r_.y(0) = new_obstacle.radius;

  kf_x_.correctState();
  kf_y_.correctState();
  kf_r_.correctState();

  obstacle_.center.x = kf_x_.q_est(0);
  obstacle_.center.y = kf_y_.q_est(0);

  obstacle_.velocity.x = kf_x_.q_est(1);
  obstacle_.velocity.y = kf_y_.q_est(1);

  obstacle_.radius = kf_r_.q_est(0);

  fade_counter_ = s_fade_counter_size_;   // 관측되면 수명 리셋
}
```

**KF 수식** — [kalman.h:64-76](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/kalman.h#L64-L76)

```cpp
void predictState() {
  q_pred = A * q_est + B * u;
  P = A * P * trans(A) + Q;
}

void correctState() {
  K = P * trans(C) * inv(C * P * trans(C) + R);
  q_est = q_pred + K * (y - C * q_pred);
  P = (I - K * C) * P;
}
```

**행렬 설정** — [tracked_circle_obstacle.h:162-210](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L162-L210)

```cpp
void initKF() {
  kf_x_.A(0, 1) = s_sampling_time_;   // A = [[1, dt], [0, 1]]  등속 모델
  ...
  kf_x_.C(0, 0) = 1.0;                // C = [1, 0]  위치만 관측
  ...
  kf_x_.R(0, 0) = s_measurement_variance_;
  kf_x_.Q(0, 0) = s_process_variance_;
  kf_x_.Q(1, 1) = s_process_rate_variance_;
  ...
  // P-2 two-point initiation covariance (Bar-Shalom): position known to
  // one measurement, velocity to a two-measurement difference. Upstream's
  // P = I neither matches the seed uncertainty nor the configured R, which
  // made the filter ignore position innovations while the (noisy) seeded
  // velocity converged.
  if (!s_legacy_init_ && s_sampling_time_ > 0.0) {
    const double vel_var =
        2.0 * s_measurement_variance_ / (s_sampling_time_ * s_sampling_time_);
    for (KalmanFilter* kf : {&kf_x_, &kf_y_, &kf_r_}) {
      kf->P(0, 0) = s_measurement_variance_;
      ...
```

가변 `dt` 대응 — [tracked_circle_obstacle.h:150-160](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L150-L160)

```cpp
// P-2: measurement-driven updates arrive with variable dt (from header
// stamps); refresh the state-transition and process-noise terms before
// every prediction instead of freezing them at construction time.
void applyTimestep() {
  const double scale =
      (s_nominal_dt_ > 0.0) ? s_sampling_time_ / s_nominal_dt_ : 1.0;
  for (KalmanFilter* kf : {&kf_x_, &kf_y_, &kf_r_}) {
    kf->A(0, 1) = s_sampling_time_;
    kf->Q(0, 0) = s_process_variance_ * scale;
    kf->Q(1, 1) = s_process_rate_variance_ * scale;
  }
}
```

**트랙 수명** — `fade_counter_`가 0이 되면 삭제
([obstacle_tracker.cpp:206-220](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L206-L220), `hasFaded()`).

#### (e) 발행 — 공분산 export 포함

[obstacle_tracker.cpp:820-848](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L820-L848)

```cpp
for (auto& tracked_circle_obstacle : tracked_circle_obstacles_) {
  obstacle_detector::msg::CircleObstacle ob = tracked_circle_obstacle.getObstacle();
  ob.true_radius = ob.radius - radius_margin_;
  // P-3: export the per-track posterior estimate-error variances. Only
  // meaningful because P-2 initialises P from R (diag(R, 2R/dt^2)) instead of
  // upstream's P = I; before that these numbers were an arbitrary unit.
  // Consumed by safety_obstacle_filter's k_sigma inflation term (§9.6, H-8).
  ob.covariance[0] = tracked_circle_obstacle.getKFx().P(0, 0);
  ob.covariance[1] = tracked_circle_obstacle.getKFy().P(0, 0);
  ob.covariance[2] = tracked_circle_obstacle.getKFr().P(0, 0);
  ...
```

[obstacle_tracker.cpp:869-875](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L869-L875)

```cpp
// Measurement-driven mode stamps with the measurement time — downstream
// staleness logic and replay determinism both depend on it. A-mode stamps
// with now() like upstream: the timer state has been advanced past the
// last measurement, so its stamp would be a lie there.
obstacles_msg.header.stamp = (!timerMode() && have_measurement_)
    ? last_measurement_stamp_ : rclcpp::Time(nh_->get_clock()->now());
```

### 3.5 [5단계] 안전 게이팅 + 반경 팽창 ★

파일: [gating.h](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h)

[gating.h:87-135](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h#L87-L135)

```cpp
inline obstacle_detector::msg::Obstacles Apply(
    const obstacle_detector::msg::Obstacles& in, const Params& p,
    double now_s, Stats* stats = nullptr) {
  obstacle_detector::msg::Obstacles out;
  out.header = in.header;
  const double stamp_s = static_cast<double>(in.header.stamp.sec) +
                         1e-9 * static_cast<double>(in.header.stamp.nanosec);
  if (now_s - stamp_s > p.max_age) {
    if (stats) ++stats->stale_messages;
    return out;  // stale: gate every circle (stamp passes through)
  }
  out.circles.reserve(in.circles.size());
  for (const auto& c : in.circles) {
    if (c.true_radius > p.max_circle_radius) {         // ① 과대 반경 드롭
      if (stats) { ++stats->dropped_large_radius; ... }
      continue;
    }
    auto safe = c;
    const double r = std::max(c.true_radius, p.min_radius);   // ② 최소 반경 하한
    double vx = c.velocity.x;
    double vy = c.velocity.y;
    const double speed = std::hypot(vx, vy);
    if (speed > p.v_max_obstacle) {                    // ③ 속도 클램프
      const double k = p.v_max_obstacle / speed;
      vx *= k;
      vy *= k;
    }
    safe.velocity.x = vx;
    safe.velocity.y = vy;
    double sigma_term = 0.0;
    if (p.use_covariance) {                            // ④ σ 항 (기본 OFF)
      const double sigma = SurfaceSigma(c);
      ...
      sigma_term = p.k_sigma * std::min(sigma, p.sigma_max);
    }
    safe.radius = r + p.fixed_inflation + sigma_term +  // ⑤ 최종 안전 반경
                  std::hypot(vx, vy) * p.latency_horizon;
    out.circles.push_back(safe);
  }
  return out;
}
```

**안전 반경 공식**

```
radius_safe = max(true_radius, min_radius)
            + fixed_inflation
            + k_sigma · min(σ, sigma_max)      ← use_covariance=true 일 때만
            + |v| · latency_horizon
```

σ 계산 — [gating.h:80-85](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h#L80-L85)

```cpp
inline double SurfaceSigma(const obstacle_detector::msg::CircleObstacle& c) {
  const double var_pos = std::max(c.covariance[0], c.covariance[1]);
  const double var_r = c.covariance[2];
  const double var = var_pos + var_r;
  return (var > 0.0) ? std::sqrt(var) : 0.0;
}
```

**드롭은 반드시 관측 가능하게** — [safety_obstacle_filter_node.cpp:62-86](../ros2/src/g1_perception/safety_obstacle_filter/src/safety_obstacle_filter_node.cpp#L62-L86)

```cpp
// Every path on which this node silently changes or discards an obstacle
// gets a counter and a throttled warning. Gap G2's whole lesson is that a
// drop nobody can observe is a defect wherever the threshold sits ...
void ReportStats() {
  if (stats_.dropped_large_radius != last_.dropped_large_radius) {
    RCLCPP_WARN_THROTTLE(..., "dropped %ld circle(s) with true_radius > "
        "max_circle_radius %.2f m (largest %.3f m) — INVISIBLE to DPCBF", ...);
  }
  ...
```

### 3.6 [6단계] 제어 측 수신 — 버퍼 · 외삽 · staleness ★

#### (a) 무잠금 더블버퍼 (seqlock)

10 Hz 쓰기(ROS executor 스레드) ↔ 1 kHz 읽기(제어 스레드).

[obstacle_buffer.h:126-185](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L126-L185)

```cpp
// Wait-free single-writer (executor thread, ~10 Hz) / single-reader (1 kHz
// bridge thread) frame exchange: double buffer, each slot seqlock-guarded.
// The reader never blocks; a retry can only happen if the writer laps the
// reader's slot mid-copy, which at 10 Hz writes vs µs reads requires the
// reader to stall >100 ms inside the copy.
class ObstacleBuffer {
 public:
  void Publish(const ObstacleFrame& frame) {
    const std::uint32_t next = 1u - front_.load(std::memory_order_relaxed);
    Slot& slot = slots_[next];
    slot.seq.fetch_add(1, std::memory_order_release);  // odd: writing
    ...
    slot.seq.fetch_add(1, std::memory_order_release);  // even: done
    front_.store(next, std::memory_order_release);
    frames_published_.fetch_add(1, std::memory_order_relaxed);
  }
```

#### (b) staleness 사다리

[obstacle_buffer.h:31-64](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L31-L64)

```cpp
enum class StalenessState : std::uint32_t {
  kFresh = 0,        // age <= max_age
  kDegrade = 1,      // max_age < age <= max_age + fade_out: command ramps to 0
  kStop = 2,         // age beyond fade_out: command 0, retained set inflated
  kNoData = 3,       // never received a frame
};

struct StalenessPolicy {
  double max_age_s = 0.30;
  double fade_out_s = 0.30;
  double hold_after_stale_s = 1.0;  // inflation growth cap (§10.3)

  StalenessState Classify(double age_s) const {
    if (age_s <= max_age_s) return StalenessState::kFresh;
    if (age_s <= max_age_s + fade_out_s) return StalenessState::kDegrade;
    return StalenessState::kStop;
  }

  // 1 while fresh, linear ramp to 0 across the fade-out window, 0 after.
  // Applied by the CALL SITE to the desired command before Filter() (§10.5).
  double CommandScale(double age_s) const {
    if (age_s <= max_age_s) return 1.0;
    if (age_s >= max_age_s + fade_out_s) return 0.0;
    return 1.0 - (age_s - max_age_s) / fade_out_s;
  }
};
```

| 상태 | 조건 (age) | 명령 스케일 | 장애물 집합 |
|---|---|---|---|
| `kFresh` | ≤ 0.30 s | 1.0 | 그대로 |
| `kDegrade` | 0.30 ~ 0.60 s | 1.0 → 0.0 선형 감쇠 | 등속 외삽 |
| `kStop` | > 0.60 s | 0.0 | **유지** + 반경 팽창 (최대 1.0 s 분) |
| `kNoData` | 프레임 수신 이력 없음 | 0.0 | 빈 집합 |

#### (c) 외삽(extrapolation) — 10 Hz를 1 kHz 제어 시각으로

[obstacle_buffer.h:89-124](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L89-L124)

```cpp
inline void Materialize(const ObstacleFrame& frame, double t_query_s,
                        const StalenessPolicy& policy,
                        MaterializedSnapshot* out) {
  out->obstacles.clear();
  if (frame.stamp < 0.0) {
    out->fresh = false;
    out->age_s = std::numeric_limits<double>::infinity();
    out->command_scale = 0.0;
    out->state = StalenessState::kNoData;
    return;
  }
  // A stamp minimally ahead of the query (pipeline stages share sim time but
  // sample it asynchronously) is treated as age 0, never negative dt.
  const double age = std::max(0.0, t_query_s - frame.stamp);
  const StalenessState state = policy.Classify(age);
  const double dt_extrap =
      std::min(age, policy.max_age_s + policy.fade_out_s);
  const double inflate_horizon =
      state == StalenessState::kStop
          ? std::min(age, policy.hold_after_stale_s)
          : 0.0;
  out->obstacles.reserve(frame.count);
  for (std::size_t i = 0; i < frame.count; ++i) {
    dpcbf::ObstacleState o = frame.obstacles[i];
    o.x += o.velocity_x * dt_extrap;                 // ← 등속 외삽
    o.y += o.velocity_y * dt_extrap;
    if (inflate_horizon > 0.0) {
      o.radius += std::hypot(o.velocity_x, o.velocity_y) * inflate_horizon;
    }
    out->obstacles.push_back(o);
  }
  out->fresh = state == StalenessState::kFresh;
  out->age_s = age;
  out->command_scale = policy.CommandScale(age);
  out->state = state;
}
```

> **설계 원칙**: 데이터가 낡았다고 **장애물 집합을 비우지 않는다.**
> 비우면 순간적으로 "장애물 없음"이 되어 로봇이 돌진할 수 있다.
> 대신 집합을 유지하고 반경을 키우며, 명령 스케일을 0으로 만든다.

#### (d) 조회 진입점

[obstacle_source.cpp:337-390](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L337-L390)

```cpp
ObstacleSource::Snapshot ObstacleSource::GetObstacles(double t_query_s) {
  const auto t0 = std::chrono::steady_clock::now();
  Snapshot snap;
  Impl& im = *impl_;
  switch (im.config.mode) {
    case Mode::kOracle:
      // Pass-through: no topic, no extrapolation, no staleness (§10.4).
      snap.obstacles = im.config.oracle();
      break;
    case Mode::kShadow: { ... }   // 정답값으로 제어 + 추정값 오차 통계만 누적
    case Mode::kEstimated: {
      im.buffer.Read(&im.scratch_frame);
      Materialize(im.scratch_frame, t_query_s, im.config.staleness,
                  &im.scratch_snapshot);
      snap.obstacles = std::move(im.scratch_snapshot.obstacles);
      ...
```

조회 지연은 히스토그램으로 계측되어 `/dpcbf/status`에 실린다
([obstacle_source.cpp:379-388](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L379-L388)).

### 3.7 [7단계] DPCBF 안전 필터로 전달

[main.cc:1016-1018](../simulate/src/main.cc#L1016-L1018)

```cpp
const auto scaled = dra::ScaleDesired(desired, snap.command_scale);
const auto filtered = safety_filter.Filter(robot, scaled, snap.obstacles);
dpcbf_visualizer.Update(robot, snap.obstacles, filtered);
```

- `command_scale` 감쇠는 **`Filter()` 호출 전, 호출부에서** 적용된다 (§10.5).
- 결과는 [dpcbf_safety_filter.h:60-73](../dpcbf/include/dpcbf/dpcbf_safety_filter.h#L60-L73) `SafetyFilterResult`.

---

## 4. 데이터 구조 정리

### 4.1 ROS 메시지

[Obstacles.msg](../ros2/src/external/obstacle_detector_2/msg/Obstacles.msg)

```
std_msgs/Header header

obstacle_detector/SegmentObstacle[] segments
obstacle_detector/CircleObstacle[] circles
```

[CircleObstacle.msg](../ros2/src/external/obstacle_detector_2/msg/CircleObstacle.msg)

```
uint64 uid                      # Unique identifier
geometry_msgs/Point center      # Central point [m]
geometry_msgs/Vector3 velocity  # Linear velocity [m/s]
float64 radius                  # Radius with added margin [m]
float64 true_radius             # True measured radius [m]
string semclass                 # Semantic class
float64 confidence              # Confidence in semantic class
float64[3] covariance           # [var_x, var_y, var_r] m^2 — 트래커 사후 공분산
                                # P(0,0). 트래커가 producer가 아니면 전부 0.
```

### 4.2 어댑터 내부 프레임

[obstacle_buffer.h:22-29](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L22-L29)

```cpp
constexpr std::size_t kMaxObstacles = 128;

struct ObstacleFrame {
  double stamp = -1.0;  // sim-time seconds from header.stamp; <0 = no data yet
  std::size_t count = 0;
  std::array<dpcbf::ObstacleState, kMaxObstacles> obstacles{};
};
```

### 4.3 uid → id 매핑

[obstacle_buffer.h:66-68](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L66-L68)

```cpp
inline int UidToId(std::uint64_t uid) {
  return static_cast<int>(uid & 0x7fffffffu);
}
```

---

## 5. 파라미터 전체 정리

### 5.1 검출 — [obstacle_detector.yaml](../ros2/src/g1_perception/g1_perception_bringup/config/obstacle_detector.yaml)

```yaml
obstacle_extractor:
  ros__parameters:
    active: true
    use_scan: true
    use_pcl: false
    min_group_points: 5
    max_group_distance: 0.10
    distance_proportion: 0.01745
    max_split_distance: 0.20
    max_merge_separation: 0.20
    max_merge_spread: 0.20
    max_circle_radius: 0.60        # 센싱 한계 (피팅 반경 기준, 패치 0009)
    radius_enlargement: 0.17       # 0.25에서 재튜닝 (short-arc bias, H-7)
    circles_from_visibles: true
    discard_converted_segments: true
    transform_coordinates: true
    frame_id: odom
```

| 파라미터 | 값 | 의미 |
|---|---|---|
| `min_group_points` | 5 | 그룹으로 인정할 최소 점 개수 |
| `max_group_distance` | 0.10 m | 그룹 분리 기본 임계값 |
| `distance_proportion` | 0.01745 | 거리 비례 항 (≈1° in rad) |
| `max_circle_radius` | 0.60 m | 이보다 큰 원은 **드롭** (완전 가시 실효 한계 ≈ 0.55 m true radius) |
| `radius_enlargement` | 0.17 m | 짧은 원호 피팅 바이어스 보정 |
| `circles_from_visibles` | true | 완전히 보이는 선분만 원으로 변환 |

### 5.2 추적 — 같은 파일

```yaml
obstacle_tracker:
  ros__parameters:
    active: true
    sensor_rate: 10.0                 # /scan rate; association cost + promotion catch-up
    compensate_robot_velocity: false  # 입력이 odom 프레임이므로 반드시 off
    loop_rate: 10.0
    tracking_duration: 1.0
    min_correspondence_cost: 0.3      # association gate 0.30 m
    radius_residual_weight: 0.3       # Appendix-A P-2 scope
    std_correspondence_dev: 0.15
    process_variance: 0.0001
    process_rate_variance: 0.03
    measurement_variance: 1.0         # ⚠ 아래 "알려진 이슈" 참고
    frame_id: odom
```

### 5.3 안전 필터 — [safety_obstacle_filter.yaml](../ros2/src/g1_perception/g1_perception_bringup/config/safety_obstacle_filter.yaml)

```yaml
safety_obstacle_filter:
  ros__parameters:
    max_age: 0.30              # s
    min_radius: 0.20           # m
    max_circle_radius: 0.60    # m (extractor와 반드시 동일 값 유지)
    fixed_inflation: 0.051     # m (Phase-4 실측 보정; Appendix-A 0.03에서 상향)
    latency_horizon: 0.12      # s
    v_max_obstacle: 1.5        # m/s
    use_covariance: false      # σ 항 기본 OFF
    k_sigma: 2.748             # placeholder, 미보정
    sigma_max: 0.50            # m
```

> `fixed_inflation: 0.051`의 근거: S1–S4 시나리오 정상상태 오차를 담는 최소값.
> 지배 요인은 **큰 원기둥에 대한 원 피팅 중심 바이어스**
> (r_gt=0.30 blocker: 중심 83 mm + 반경 33 mm 과대추정 → 필요 여유 최대 51 mm).
> 가림/병합 과도구간(S3 15%, S4 최대 0.38 m)은 **고정항으로 못 덮는다** — 그게 σ 항의 존재 이유.

### 5.4 어댑터 — [dpcbf_ros_adapter.yaml](../ros2/src/g1_perception/g1_perception_bringup/config/dpcbf_ros_adapter.yaml)

```yaml
dpcbf_ros_adapter:
  topic: /obstacles_safe
  max_age: 0.30              # s
  fade_out: 0.30             # s
  hold_after_stale: 1.0      # s

plot_bridge:
  enabled: true
  topic: /dpcbf/plot
  rate_hz: 30.0
  frame_id: odom
```

> 이 파일은 ROS 파라미터 파일이 **아니고** `main.cc`가 yaml-cpp로 직접 읽는다
> ([adapter_config.h](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/adapter_config.h),
> [main.cc:956-966](../simulate/src/main.cc#L956-L966)). 키가 하나라도 없으면 시작 시 요란하게 실패한다.
> 헤더의 컴파일 기본값은 이 파일과 **같게 유지**해야 한다 (t1_replay·유닛테스트 등 offline 소비자용).

---

## 6. 실행 방법

```bash
# 시뮬레이션 + 추정 모드
ros2 launch g1_perception_bringup bringup.launch.py \
    source:=sim mode:=estimated viz:=rviz

# 하드웨어
ros2 launch g1_perception_bringup bringup.launch.py \
    source:=hw mode:=estimated
```

| 인자 | 값 | 설명 |
|---|---|---|
| `source` | `sim` \| `hw` | 센서 소스. `hw`면 `use_sim_time`이 자동으로 false |
| `mode` | `oracle` \| `shadow` \| `estimated` | 어댑터 모드 (§2.6) |
| `viz` | `off` \| `rviz` | 시각화 |
| `record` | `off` \| `on` | rosbag 기록 |
| `voxel` | `off` \| `on` | VoxelGrid 다운샘플 삽입 (CPU 여유 없을 때) |
| `ground_seg` | ~~`patchwork`~~ | **미구현 — 지정 시 에러** |

---

## 7. 디버깅 / 확인 방법

### 7.1 토픽 확인

```bash
ros2 topic hz   /scan /raw_obstacles /tracked_obstacles /obstacles_safe
ros2 topic echo /obstacles_safe --once
ros2 topic echo /dpcbf/status          # 어댑터 진단 (10 Hz)
```

`/dpcbf/status`에 실리는 키
([obstacle_source.cpp:104-194](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L104-L194)):

| 키 | 의미 |
|---|---|
| `mode` | oracle / shadow / estimated |
| `staleness_state` | fresh / degrade / stop / no_data |
| `age_s` | 마지막 프레임 나이 |
| `command_scale` | 현재 명령 감쇠 계수 |
| `frames_received` | 누적 수신 프레임 수 |
| `dropped_circles` | 128개 초과로 버린 원 개수 |
| `query_latency_p50_le_us` / `p99` | 조회 지연 백분위 |
| `shadow_pos_err_mean_m` 등 | shadow 모드 오차 통계 |

### 7.2 보조 스크립트

[g1_perception_bringup/scripts/](../ros2/src/g1_perception/g1_perception_bringup/scripts/)

| 스크립트 | 용도 |
|---|---|
| `hw_obstacle_watch.py` | 실기 장애물 스트림 실시간 관찰 |
| `hw_diagnostics.py` | 하드웨어 진단 종합 |
| `hw_source_probe.py`, `hw_tf_probe.py` | 센서/TF 개별 점검 |
| `hw_odom_drift.py` | odom 드리프트 측정 |
| `g1_hw_preflight.sh` | 실기 구동 전 점검 |
| `config_diff.py` | 설정 파일 차이 비교 |

### 7.3 시각화

- RViz 마커 릴레이: [obstacles_marker_relay.cpp](../ros2/src/g1_perception/g1_perception_utils/src/obstacles_marker_relay.cpp)
- DPCBF 오버레이: [dpcbf_overlay_node.cpp](../ros2/src/g1_perception/g1_perception_utils/src/dpcbf_overlay_node.cpp)
- 원격 플로팅(Computer 3): `/dpcbf/plot` 30 Hz →
  [dpcbf_plot_client](../ros2/src/g1_perception/dpcbf_plot_client/)

---

## 8. 알려진 이슈 / 주의사항

### 8.1 ⚠ `measurement_variance: 1.0` 은 잘못된 상속값

[obstacle_detector.yaml](../ros2/src/g1_perception/g1_perception_bringup/config/obstacle_detector.yaml) 주석 요약:

- 단위는 **m²**이고 KF의 `R(0,0)`이다. 관측이 미터 단위 원 중심이므로 `sqrt(P(0,0))`도 미터.
- 값 `1.0`은 **1-sigma가 1 m인 LiDAR**를 주장하는 셈이다.
- P-2가 `P(0,0) = R`로 초기화하므로 모든 트랙이 σ = 1.0 m로 태어나 ~0.58 m로 수렴한다.
- 이 상태에서 `k_sigma`를 맞추면 오차를 조용히 흡수해 버린다.
- s1_surveyed 실측 산포는 1.775e-06 m² (1σ = 1.3 mm)지만 **잡음 없는 해석적 레이캐스트**라
  이것도 출하값이 아니다.
- **실기 데이터(5B block 1)에서 R을 도출한 뒤에** `k_sigma`를 보정해야 한다.

### 8.2 ⚠ σ 항(`use_covariance`)은 기본 OFF

- 경로는 살아 있으나 `k_sigma: 2.748`은 **폐기된 브랜치의 값**이라 placeholder다.
- 켜는 것은 `(fixed_inflation, k_sigma)` **동시 재보정**을 의미한다 — 고정항이 이미 원 피팅
  정상상태 바이어스를 덮고 있어 σ가 이중 계상된다.
- 잘못된 σ로 켜면 시뮬 커버리지가 92.5% → 99.9%로 올라가지만, **처방의 형태만 맞고 크기는 미보정**이다.

### 8.3 ⚠ 지면 분할 없음

바닥 반사를 거르는 건 `pointcloud_to_laserscan`의 `min_height`뿐이다.
평평한 바닥에서만 유효한 **높이 밴드**이지 분할이 아니다.

> 부수 발견: [bringup.launch.py:40](../ros2/src/g1_perception/g1_perception_bringup/launch/bringup.launch.py#L40)의
> 에러 메시지는 `min_height (0.15 m in base_footprint)`라고 안내하지만, 실제
> [pointcloud_to_laserscan.yaml](../ros2/src/g1_perception/g1_perception_bringup/config/pointcloud_to_laserscan.yaml)
> 값은 **0.70 m**다. 메시지가 오래되어 어긋난 상태 — 값을 판단할 땐 yaml을 믿을 것.

### 8.4 ⚠ `compensate_robot_velocity`는 반드시 false

트래커 입력이 이미 **odom 프레임**(extractor가 변환)인데, 보상 코드는 robot 프레임 입력을
가정한다 ([obstacle_tracker.cpp:838-846](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L838-L846)).

### 8.5 ⚠ 두 곳의 `max_circle_radius`는 같이 움직여야 한다

`obstacle_detector.yaml`(0.60)과 `safety_obstacle_filter.yaml`(0.60)이 나뉘어 있는 것이
gap G2가 발생한 원인이다. 한쪽만 바꾸면 안 된다.

### 8.6 ⚠ ROS 2 Foxy 호환

G1 온보드는 Foxy다. 그래서 구독 콜백이 `const Msg&`가 아니라 `ConstSharedPtr`다
(const-reference 콜백은 Galactic 이상). 코드 수정 시 이 패턴을 유지할 것.

---

## 9. 빠른 참조 — 파일별 역할

| 파일 | 역할 |
|---|---|
| [perception.launch.py](../ros2/src/g1_perception/g1_perception_bringup/launch/perception.launch.py) | 파이프라인 전체 배선 (한 곳) |
| [obstacle_extractor.cpp](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp) | 스캔 → 선분 → 원 검출 |
| [obstacle_tracker.cpp](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp) | 데이터 연관 + 트랙 관리 |
| [tracked_circle_obstacle.h](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h) | 트랙당 x/y/r 칼만필터 |
| [kalman.h](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/kalman.h) | KF 수식 (armadillo) |
| [gating.h](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h) | 게이팅 + 안전 반경 팽창 |
| [safety_obstacle_filter_node.cpp](../ros2/src/g1_perception/safety_obstacle_filter/src/safety_obstacle_filter_node.cpp) | 위 로직의 ROS 노드 래퍼 |
| [obstacle_source.cpp](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp) | **구독 + 진단 + 모드 분기** |
| [obstacle_buffer.h](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h) | 무잠금 버퍼 + 외삽 + staleness |
| [main.cc:1008~](../simulate/src/main.cc#L1008) | 제어루프에서 조회 → DPCBF |
| [dpcbf_safety_filter.h](../dpcbf/include/dpcbf/dpcbf_safety_filter.h) | DPCBF 자료형/인터페이스 |

### "한 곳만 본다면"

- **받아오는 코드** → [obstacle_source.cpp:82-102](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L82-L102) `OnObstacles()`
- **추정 본체** → [obstacle_tracker.cpp:268-388](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L268-L388) `obstaclesCallbackCircles()`
