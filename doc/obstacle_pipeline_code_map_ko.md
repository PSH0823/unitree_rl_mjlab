# 장애물 파이프라인 코드 맵 — 파일 관계 · 변수 계보

두 가지 질문에만 집중한 문서다.

1. **장애물 위치를 받아오는 코드는 어디인가** — 퍼블리셔 / 섭스크라이버 / 받아온 변수 이름
2. **장애물을 추정하는 과정의 핵심 코드는 어디인가**

파라미터 값·튜닝 근거·디버깅 절차는 [obstacle_estimation_pipeline_ko.md](obstacle_estimation_pipeline_ko.md)에 따로 있다.
이 문서는 **"어느 파일이 어느 파일과 어떻게 이어지고, 어느 변수가 어느 변수로 바뀌는가"**만 다룬다.

---

# Part A. 파일 간의 관계

## A-1. 세 종류의 연결

파일들은 서로 **세 가지 방식**으로만 이어져 있다. 이걸 구분하지 않으면 지도가 안 그려진다.

| 연결 방식 | 표기 | 의미 |
|---|---|---|
| **토픽** | `═══▶` | 프로세스/노드 경계. DDS 또는 intra-process |
| **include** | `───▶` | 컴파일 타임 의존. 타입을 공유 |
| **함수 호출** | `··▶` | 같은 번역 단위 안 |

## A-2. 토픽으로 이어진 관계 (노드 경계)

```
 sim_mjlidar_bridge / livox_ros_driver2
        ║ /livox/lidar                          (sensor_msgs/PointCloud2)
        ▼
 ┌──────────────────────────────────────────────────────────────┐
 │  perception_container  (컴포저블 노드 1개 프로세스)             │
 │                                                              │
 │  pcl_ros::CropBox                                            │
 │        ║ /points_self_filtered                               │
 │        ▼                                                     │
 │  pointcloud_to_laserscan                                     │
 │        ║ /scan                       (sensor_msgs/LaserScan) │
 │        ▼                                                     │
 │  obstacle_extractor.cpp        ◀── obstacle_detector.yaml    │
 │        ║ /raw_obstacles       (obstacle_detector/Obstacles)  │
 │        ▼                                                     │
 │  obstacle_tracker.cpp          ◀── obstacle_detector.yaml    │
 │        ║ /tracked_obstacles                                  │
 │        ▼                                                     │
 │  safety_obstacle_filter_node.cpp ◀ safety_obstacle_filter.yaml│
 │        ║ /obstacles_safe                                     │
 └────────╫─────────────────────────────────────────────────────┘
          ║  ← 여기서만 DDS를 실제로 탄다
          ▼
 ┌────────────────────────────────────────────────────────────┐
 │  simulate (제어 프로세스)                                    │
 │                                                            │
 │  obstacle_source.cpp           ◀── dpcbf_ros_adapter.yaml  │
 │        ·· GetObstacles()                                   │
 │        ▼                                                   │
 │  main.cc  (1 kHz 제어루프)                                  │
 │        ·· Filter()                                         │
 │        ▼                                                   │
 │  dpcbf_safety_filter                                       │
 └────────────────────────────────────────────────────────────┘
```

이 배선은 **전부 한 파일에서** 정의된다:
[perception.launch.py:30-91](../ros2/src/g1_perception/g1_perception_bringup/launch/perception.launch.py#L30-L91)

```python
extractor = ComposableNode(
    package='obstacle_detector',
    plugin='obstacle_detector::ObstacleExtractorComponent',
    remappings=[('scan',          '/scan'),
                ('raw_obstacles', '/raw_obstacles')],   # ← 출력이
    ...)
tracker = ComposableNode(
    plugin='obstacle_detector::ObstacleTrackerComponent',
    remappings=[('raw_obstacles',     '/raw_obstacles'),   # ← 다음 입력으로
                ('tracked_obstacles', '/tracked_obstacles')],
    ...)
safety_filter = ComposableNode(
    plugin='safety_obstacle_filter::SafetyObstacleFilterNode',
    remappings=[('tracked_obstacles', '/tracked_obstacles'),
                ('obstacles_safe',    '/obstacles_safe')],
    ...)
```

> **핵심**: `[1]~[5]`는 한 컨테이너 안에서 `use_intra_process_comms: True`로 돈다.
> 즉 노드 간 직렬화가 없다. **DDS를 실제로 타는 건 `/obstacles_safe` 하나뿐**이고,
> 그래서 staleness(나이) 관리가 이 지점에 집중되어 있다.

## A-3. include로 이어진 관계 (타입 공유)

```
                     dpcbf/include/dpcbf/dpcbf_safety_filter.h
                     ┌──────────────────────────────────────┐
                     │  struct ObstacleState { x,y,radius,  │  ← rclcpp 없음
                     │       velocity_x, velocity_y, id }   │     ROS 없음
                     │  struct RobotState / VelocityCommand │     순수 C++
                     │  class  DpcbfSafetyFilter            │
                     └──────────────────────────────────────┘
                          ▲                        ▲
                          │ include                │ include
                          │                        │
   obstacle_buffer.h ─────┘                        └───── main.cc
   ┌──────────────────────────────┐
   │ ObstacleFrame                │  ← rclcpp 없음, ROS 메시지도 없음
   │ StalenessPolicy / State      │     ⇒ 유닛테스트가 executor 없이 가능
   │ Materialize()                │
   │ ObstacleBuffer (seqlock)     │
   │ UidToId()                    │
   └──────────────────────────────┘
                          ▲
                          │ include
   obstacle_source.h ─────┘
   ┌──────────────────────────────┐
   │ class ObstacleSource         │  ← rclcpp 없음! (pimpl로 숨김)
   │   Mode / Config / Snapshot   │     ⇒ main.cc가 rclcpp 없이 include 가능
   │   struct Impl;  ← 선언만     │
   └──────────────────────────────┘
                          ▲
                          │ include (구현)
   obstacle_source.cpp ───┘
   ┌──────────────────────────────┐
   │ #include <rclcpp/rclcpp.hpp> │  ← rclcpp가 등장하는 첫 지점
   │ #include <obstacle_detector/ │
   │            msg/obstacles.hpp>│
   │ struct ObstacleSource::Impl  │
   └──────────────────────────────┘
```

[main.cc:46-52](../simulate/src/main.cc#L46-L52)가 그 이유를 직접 설명한다:

```cpp
// Both headers are rclcpp-free; ObstacleSource itself links rclcpp and is
// only used in the ROS2 build.
#include "dpcbf_ros_adapter/dpcbf_seam.h"
#include "dpcbf_ros_adapter/filter_io_log.h"
// Header is rclcpp-free; the OFF build uses only the Mode enum (no
// ObstacleSource is constructed, so nothing links against the adapter lib).
#include "dpcbf_ros_adapter/obstacle_source.h"
```

같은 원리가 safety filter에도 적용되어 있다:

```
   gating.h                          safety_obstacle_filter_node.cpp
   ┌────────────────────────────┐    ┌────────────────────────────────┐
   │ #include <obstacle_detector│    │ #include <rclcpp/rclcpp.hpp>   │
   │           /msg/obstacles>  │◀───│ #include "…/gating.h"          │
   │ struct Params / Stats      │    │ class SafetyObstacleFilterNode │
   │ SurfaceSigma()             │    │   → 파라미터 선언 + pub/sub만  │
   │ Apply()   ← 규칙 전부      │    │   → 규칙은 하나도 없음         │
   │ rclcpp 없음 = 노드 없이 테스트│  └────────────────────────────────┘
   └────────────────────────────┘
```

**패턴 요약**: 이 저장소는 일관되게 **"규칙은 헤더(ROS-free/node-free), 배선은 .cpp"**로 나눠져 있다.

| 헤더 (규칙, 테스트 가능) | .cpp (배선, ROS 필요) |
|---|---|
| [gating.h](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h) `Apply()` | [safety_obstacle_filter_node.cpp](../ros2/src/g1_perception/safety_obstacle_filter/src/safety_obstacle_filter_node.cpp) pub/sub |
| [obstacle_buffer.h](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h) `Materialize()` | [obstacle_source.cpp](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp) 구독 + 진단 |

## A-4. 설정 파일 → 어느 변수를 채우는가

```
obstacle_detector.yaml
   ├─ obstacle_extractor:  ─▶ ObstacleExtractor::updateParamsUtil()  → p_max_group_distance_,
   │                            [obstacle_extractor.cpp:67]            p_max_circle_radius_,
   │                                                                   p_radius_enlargement_ …
   └─ obstacle_tracker:    ─▶ ObstacleTracker::updateParamsUtil()   → p_min_correspondence_cost_,
                                [obstacle_tracker.cpp:73]              s_measurement_variance_ …
                                                                       ↓ static setter
                                                          TrackedCircleObstacle::setCovariances()

safety_obstacle_filter.yaml
   └─ safety_obstacle_filter: ─▶ declare_parameter() 7회 → params_ (struct Params)
                                  [safety_obstacle_filter_node.cpp:16-32]
                                       ↓ 인자로 전달
                                  Apply(msg, params_, now().seconds(), &stats_)

dpcbf_ros_adapter.yaml            ← ROS 파라미터 파일이 아님! yaml-cpp로 직접 파싱
   ├─ dpcbf_ros_adapter: ─▶ dra::LoadAdapterConfig()  → source_config.topic
   │                          [main.cc:961]              source_config.staleness.{max_age_s,
   │                                                       fade_out_s, hold_after_stale_s}
   └─ plot_bridge:       ─▶ dra::LoadVizBridgeConfig() → viz_config
                              [main.cc:976]
```

## A-5. 파일별 "무엇을 무엇으로 바꾸는가"

| 파일 | 입력 자료형 | 출력 자료형 |
|---|---|---|
| [obstacle_extractor.cpp](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp) | `LaserScan` | `Obstacles` (circles, **속도 = 0**) |
| [obstacle_tracker.cpp](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp) | `Obstacles` | `Obstacles` (**속도·uid·공분산 채워짐**) |
| [gating.h](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h) | `Obstacles` | `Obstacles` (**radius 팽창됨**) |
| [obstacle_source.cpp](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp) | `Obstacles` | `ObstacleFrame` → **`std::vector<dpcbf::ObstacleState>`** |
| [obstacle_buffer.h](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h) | `ObstacleFrame` + `t_query` | `MaterializedSnapshot` (**시간 외삽됨**) |

> 자료형이 `Obstacles`로 세 번 반복되는 게 중요하다. **타입은 같은데 의미가 매번 다르다.**
> 어느 토픽에서 뽑았는지에 따라 `velocity`가 0일 수도, `covariance`가 0일 수도 있다.

## A-6. 함수 호출 관계 (파일 내부)

```
obstacle_extractor.cpp
  scanCallback(:160)
      └·· processPoints(:219)
              ├·· groupPoints(:236) ──·· detectSegments(:287) ──·· fitSegment()  [figure_fitting.h]
              ├·· mergeSegments(:343) ─·· compareSegments(:359) ·· checkSegmentsProximity(:383)
              │                                                 ·· checkSegmentsCollinearity(:390)
              ├·· detectCircles(:397)  ──·· Circle(const Segment&)  [circle.h:53]
              ├·· mergeCircles(:448) ───·· compareCircles(:464)
              ├·· transformObstacles(:586)   ← TF: base_frame → odom
              └·· publishObstacles(:622)     ← Circle → CircleObstacle 변환

obstacle_tracker.cpp
  obstaclesCallback(:222)
      ├·· [P-2 모드] predictState()  전 트랙        [tracked_circle_obstacle.h:56]
      ├·· obstaclesCallbackCircles(:268)      ★ 추정의 심장
      │       ├·· calculateCostMatrix(:541) ··· obstacleCostFunction(:505)
      │       ├·· calculateRowMinIndices(:585) / calculateColMinIndices(:614)
      │       ├·· fuseObstacles(:697)      ← 융합
      │       ├·· fissureObstacle(:790)    ← 분열
      │       └·· correctState(new)        ← 1:1 매칭 → KF 보정
      │                                       [tracked_circle_obstacle.h:73]
      └·· publishObstacles(:820)           ← KF 상태 → CircleObstacle + covariance
  timerCallback(:197)   ← A-mode 전용
      └·· updateObstacles(:206) ··· updateState()   [tracked_circle_obstacle.h:93]

obstacle_source.cpp
  [ROS executor 스레드, ~10 Hz]        [제어 스레드, 1 kHz]
  OnObstacles(:82)                     GetObstacles(:337)
      └·· buffer.Publish(frame)             ├·· buffer.Read(&scratch_frame)
              [obstacle_buffer.h:133]       └·· Materialize(...)
                                                    [obstacle_buffer.h:89]
       └────────── seqlock 더블버퍼로만 연결 ────────┘
```

---

# Part B. 변수 간의 관계 — 변수 계보

## B-1. 전체 계보 한 장

`장애물의 x 좌표` 하나가 어떤 이름으로 살아가는지 끝까지 따라간 것이다.

```
 ┌ obstacle_extractor.cpp ────────────────────────────────────────────────┐
 │                                                                        │
 │  scan_msg.ranges[i], phi                                    :160-171   │
 │        │  Point::fromPoolarCoords(r, phi)                              │
 │        ▼                                                               │
 │  input_points_  (std::list<Point>)                                     │
 │        │  groupPoints()                                     :236       │
 │        ▼                                                               │
 │  point_set  (PointSet{begin, end, num_points, is_visible})             │
 │        │  detectSegments() → Segment(*begin, *end) 또는 fitSegment()   │
 │        ▼                                                    :287       │
 │  segments_  (std::list<Segment>{first_point, last_point})              │
 │        │  Circle(const Segment& s)                  [circle.h:53-57]   │
 │        │     radius = 0.5773502 * s.length()          ← √3/3            │
 │        │     center = (s.first + s.last - radius*s.normal()) / 2       │
 │        ▼                                                               │
 │  circle.radius                                              :424       │
 │        │  += p_radius_enlargement_  (0.17)                             │
 │        ▼                                                               │
 │  circles_  (std::list<Circle>{center, radius})                         │
 │        │  transformObstacles(): c.center = transformPoint(...)  :611   │
 │        │     ← 여기서 base_footprint → odom 프레임                      │
 │        ▼                                                               │
 │  CircleObstacle circle                                      :642-651   │
 │     circle.center.x  = c.center.x                                      │
 │     circle.velocity.x = 0.0            ★ 추출기는 속도를 모른다         │
 │     circle.radius     = c.radius                                       │
 │     circle.true_radius = c.radius - p_radius_enlargement_              │
 └────────────────────────────────────────────┬───────────────────────────┘
                              /raw_obstacles  ║
 ┌────────────────────────────────────────────▼───────────────────────────┐
 │ obstacle_tracker.cpp                                                   │
 │                                                                        │
 │  new_obstacles->circles[n]                                             │
 │        │  radius_margin_ = circles[0].radius - circles[0].true_radius  │
 │        │                                                    :270       │
 │        │  ┌ 매칭 실패 ─▶ untracked_circle_obstacles_  (대기)  :344     │
 │        │  │                    │ 다음 프레임에 승격 (2점 초기화)       │
 │        │  │                    ▼                                       │
 │        └──┴─▶ tracked_circle_obstacles_[t]  (TrackedCircleObstacle)    │
 │                     │                                                  │
 │                     │  correctState(new)     [tracked_circle_obstacle.h│
 │                     │     kf_x_.y(0) = new.center.x           :73-91]  │
 │                     │     kf_x_.correctState()      [kalman.h:70]      │
 │                     │        K = P Cᵀ (C P Cᵀ + R)⁻¹                   │
 │                     │        q_est = q_pred + K (y - C q_pred)         │
 │                     │        P = (I - K C) P                           │
 │                     ▼                                                  │
 │              obstacle_.center.x   = kf_x_.q_est(0)                     │
 │              obstacle_.velocity.x = kf_x_.q_est(1)  ★ 속도가 처음 생김  │
 │              obstacle_.radius     = kf_r_.q_est(0)                     │
 │              obstacle_.uid        = uid_next_++     ★ ID가 처음 생김    │
 │                     │  publishObstacles()                    :820-833  │
 │                     ▼                                                  │
 │              ob.true_radius   = ob.radius - radius_margin_             │
 │              ob.covariance[0] = getKFx().P(0,0)   ★ 불확실성 export     │
 │              ob.covariance[1] = getKFy().P(0,0)                        │
 │              ob.covariance[2] = getKFr().P(0,0)                        │
 └────────────────────────────────────────────┬───────────────────────────┘
                          /tracked_obstacles  ║
 ┌────────────────────────────────────────────▼───────────────────────────┐
 │ gating.h :: Apply()                                          :87-135   │
 │                                                                        │
 │  in.circles[i]  ──▶  safe  (복사본)                                    │
 │     r  = max(c.true_radius, min_radius)                      :109      │
 │     vx = clamp(c.velocity.x, v_max_obstacle)                 :113      │
 │     sigma = SurfaceSigma(c)  = √(max(cov[0],cov[1]) + cov[2])  :80     │
 │                                                                        │
 │     safe.radius = r + fixed_inflation                        :130      │
 │                     + k_sigma·min(sigma, sigma_max)   ← 기본 OFF       │
 │                     + |v|·latency_horizon                              │
 │     safe.center 는 그대로 (위치는 안 건드림)                            │
 └────────────────────────────────────────────┬───────────────────────────┘
                             /obstacles_safe  ║  ← DDS 경계
 ┌────────────────────────────────────────────▼───────────────────────────┐
 │ obstacle_source.cpp :: OnObstacles()                          :82-102  │
 │                                                                        │
 │  msg.header.stamp ──▶ frame.stamp   (double 초)                        │
 │  msg.circles[i]   ──▶ frame.obstacles[n]  (dpcbf::ObstacleState)       │
 │      c.center.x   ──▶ o.x                                              │
 │      c.center.y   ──▶ o.y                                              │
 │      c.radius     ──▶ o.radius        ★ true_radius가 아니라 팽창된 값  │
 │      c.velocity.x ──▶ o.velocity_x                                     │
 │      c.velocity.y ──▶ o.velocity_y                                     │
 │      c.uid        ──▶ o.id  = UidToId(uid) = uid & 0x7fffffff          │
 │                                                                        │
 │  buffer.Publish(frame)    ← seqlock 더블버퍼                            │
 └────────────────────────────────────────────┬───────────────────────────┘
                                              ║ 스레드 경계 (10 Hz → 1 kHz)
 ┌────────────────────────────────────────────▼───────────────────────────┐
 │ obstacle_buffer.h :: Materialize()                            :89-124  │
 │                                                                        │
 │  age = max(0, t_query_s - frame.stamp)                        :102     │
 │  dt_extrap = min(age, max_age_s + fade_out_s)                 :104     │
 │  o.x += o.velocity_x * dt_extrap        ★ 등속 외삽             :113   │
 │  o.y += o.velocity_y * dt_extrap                                       │
 │  o.radius += |v| * inflate_horizon      (kStop 상태에서만)      :116   │
 │                                                                        │
 │  out->age_s         = age                                              │
 │  out->command_scale = policy.CommandScale(age)   1.0 → 0.0             │
 │  out->state         = policy.Classify(age)                             │
 └────────────────────────────────────────────┬───────────────────────────┘
 ┌────────────────────────────────────────────▼───────────────────────────┐
 │ main.cc                                                    :1013-1017  │
 │                                                                        │
 │  auto snap = obstacle_source->GetObstacles(t_query);                   │
 │       snap.obstacles       ← std::vector<dpcbf::ObstacleState>         │
 │       snap.command_scale                                               │
 │       snap.age_s / snap.state                                          │
 │                                                                        │
 │  scaled   = dra::ScaleDesired(desired, snap.command_scale)             │
 │  filtered = safety_filter.Filter(robot, scaled, snap.obstacles)        │
 └────────────────────────────────────────────────────────────────────────┘
```

## B-2. 자료형이 바뀌는 지점만 뽑으면

| # | 자료형 | 정의 위치 | 어디서 만들어지나 |
|---|---|---|---|
| 1 | `Point` | [point.h](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/point.h) | `scanCallback` [:168](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L168) |
| 2 | `PointSet` | [point_set.h:47](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/point_set.h#L47) | `groupPoints` [:236](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L236) |
| 3 | `Segment` | [segment.h:46](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/segment.h#L46) | `detectSegments` [:291](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L291) |
| 4 | `Circle` | [circle.h:44](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/circle.h#L44) | `detectCircles` [:411](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L411) |
| 5 | `CircleObstacle` (raw) | [CircleObstacle.msg](../ros2/src/external/obstacle_detector_2/msg/CircleObstacle.msg) | `publishObstacles` [:642](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L642) |
| 6 | `TrackedCircleObstacle` | [tracked_circle_obstacle.h:48](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L48) | `obstaclesCallbackCircles` [:364](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L364) |
| 7 | `KalmanFilter` ×3 | [kalman.h:39](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/kalman.h#L39) | `TrackedCircleObstacle` 생성자 |
| 8 | `CircleObstacle` (tracked) | 같은 msg | `publishObstacles` [:827](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L827) |
| 9 | `CircleObstacle` (safe) | 같은 msg | `Apply` [:108](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h#L108) |
| 10 | `ObstacleFrame` | [obstacle_buffer.h:25](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L25) | `OnObstacles` [:83](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L83) |
| 11 | `MaterializedSnapshot` | [obstacle_buffer.h:70](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L70) | `Materialize` [:89](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L89) |
| 12 | **`dpcbf::ObstacleState`** | [dpcbf_safety_filter.h:19](../dpcbf/include/dpcbf/dpcbf_safety_filter.h#L19) | 최종 소비 자료형 |

## B-3. 필드별 계보 — "이 값은 어디서 왔나"

### `x`, `y` (위치)

| 단계 | 이름 | 무슨 일이 일어나는가 |
|---|---|---|
| 1 | `scan_msg.ranges[i]` + `phi` | 극좌표 |
| 2 | `Point.x/.y` | 직교좌표 변환 |
| 3 | `Segment.first_point/.last_point` | 선분 양 끝점 |
| 4 | `Circle.center` | **정삼각형 외접원 중심** — 선분을 밑변으로 |
| 5 | `Circle.center` (변환됨) | `base_footprint` → **`odom`** [:611](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L611) |
| 6 | `CircleObstacle.center` | 메시지로 복사 |
| 7 | `kf_x_.q_est(0)` / `kf_y_.q_est(0)` | **KF 평활화** |
| 8 | `obstacle_.center` | KF 상태에서 되읽음 |
| 9 | `safe.center` | 안전 필터는 **위치를 안 건드림** |
| 10 | `o.x` / `o.y` | `ObstacleState`로 복사 |
| 11 | `o.x += velocity_x * dt_extrap` | **시간 외삽** |
| 12 | `snap.obstacles[i].x` | DPCBF 입력 |

### `radius` — 5번 바뀐다

| 단계 | 값 | 위치 |
|---|---|---|
| 1 | `0.5773502 * segment.length()` | 원 피팅 (√3/3) [circle.h:54](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/circle.h#L54) |
| 2 | `+ radius_enlargement` (0.17) | 짧은 원호 바이어스 보정 [:426](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L426) |
| 3 | `true_radius = radius - 0.17` | 원래 피팅 값을 되돌려 저장 [:650](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L650) |
| 4 | `kf_r_.q_est(0)` | **KF 평활화** [tracked_circle_obstacle.h:88](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L88) |
| 5 | `true_radius = radius - radius_margin_` | `radius_margin_`은 첫 raw 원에서 역산 [:270](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L270), [:828](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L828) |
| 6 | `max(true_radius, min_radius) + fixed_inflation + k_sigma·σ + \|v\|·latency_horizon` | **안전 반경** [gating.h:130](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h#L130) |
| 7 | `+= \|v\| · inflate_horizon` | kStop 상태에서만 [obstacle_buffer.h:116](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L116) |

> `radius`와 `true_radius`가 계속 서로를 역산하며 오간다.
> **`radius` = 안전 여유가 포함된 값, `true_radius` = 측정된 값**이라는 규약을
> 각 단계가 자기 방식으로 다시 세운다.

### `velocity` — 추출기엔 없다

| 단계 | 값 |
|---|---|
| extractor | **`0.0`** — 단일 스캔에는 속도 정보가 없다 [:647-648](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L647-L648) |
| tracker 승격 시 | 2점 차분 시드 `(new.center - seed.center) / m_last_dt_` [:361-362](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L361-L362) |
| tracker 정상 | **`kf_x_.q_est(1)`** — KF 상태벡터의 2번째 성분 |
| safety filter | `v_max_obstacle`로 크기 클램프 (방향 보존) [gating.h:113-117](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h#L113-L117) |
| adapter | `o.velocity_x/_y`로 복사 — **외삽과 반경 팽창의 입력** |

### `id` / `uid`

| 단계 | 이름 | 타입 |
|---|---|---|
| extractor | 없음 | — |
| tracker | `obstacle_.uid = uid_next_++` [tracked_circle_obstacle.h:140](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L140) | `uint64` |
| adapter | `o.id = UidToId(c.uid) = uid & 0x7fffffff` [obstacle_buffer.h:66](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L66) | `int` |

### `header.stamp` — 안전상 가장 중요한 변수

| 단계 | 무엇으로 찍히나 |
|---|---|
| extractor | `stamp_` = **스캔 메시지의 스탬프** [:624](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L624) |
| tracker (P-2) | `last_measurement_stamp_` = **측정 시각** [:870](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L870) |
| tracker (A-mode) | `now()` — 타이머가 측정 이후로 상태를 진행시켰으므로 |
| safety filter | `out.header = in.header` — **입력 스탬프 그대로 통과** [gating.h:91](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h#L91) |
| adapter | `frame.stamp = StampToSec(msg.header.stamp)` [:84](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L84) |
| 제어 | `age = t_query_s - frame.stamp` — 여기서 staleness가 결정됨 |

> **스탬프가 처리 시각이 아니라 측정 시각으로 끝까지 전파된다.**
> 안전 필터가 자기 처리 시각으로 다시 찍으면 나이가 0으로 리셋되어
> staleness 사다리가 무력화된다. `out.header = in.header` 한 줄이 그걸 막는다.

## B-4. 스레드 경계를 넘는 변수

10 Hz ROS executor 스레드와 1 kHz 제어 스레드가 공유하는 변수는 **딱 하나**다.

```
 [executor 스레드]                        [제어 스레드]
  OnObstacles()                            GetObstacles()
       │                                        │
       │ frame (지역변수)                        │
       ▼                                        ▼
  buffer.Publish(frame)  ══▶ ObstacleBuffer ══▶ buffer.Read(&scratch_frame)
                              slots_[2]              │
                              front_ (atomic)        ▼
                              seq (atomic)      Materialize(scratch_frame, …)
                                                     │
                                                     ▼
                                                snap.obstacles
```

[obstacle_buffer.h:126-131](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L126-L131)

```cpp
// Wait-free single-writer (executor thread, ~10 Hz) / single-reader (1 kHz
// bridge thread) frame exchange: double buffer, each slot seqlock-guarded.
// The reader never blocks; a retry can only happen if the writer laps the
// reader's slot mid-copy, which at 10 Hz writes vs µs reads requires the
// reader to stall >100 ms inside the copy.
```

나머지 공유 변수는 전부 진단용 `std::atomic`이다
([obstacle_source.cpp:49-67](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L49-L67)):
`dropped_circles`, `query_count`, `last_age`, `last_scale`, `last_state`, `robot_x/y`.

`scratch_frame` / `scratch_snapshot`은 **제어 스레드 전용**이다 — `GetObstacles`가
단일 호출자라는 계약 위에서 매 틱 재할당을 피하려고 멤버로 들고 있다
([obstacle_source.cpp:69-71](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L69-L71)).

---

# Part C. 질문 1 직답 — 퍼블리셔 / 섭스크라이버 / 변수명

## C-1. 짝지어진 pub/sub 전부

| 토픽 | 퍼블리셔 (변수명 · 위치) | 섭스크라이버 (변수명 · 위치) |
|---|---|---|
| `/livox/lidar` | 외부 드라이버 | `input` (CropBox, launch remap) |
| `/points_self_filtered` | `output` (CropBox) | `cloud_in` (p2ls) |
| `/scan` | `scan` (p2ls) | **`scan_sub_`** [obstacle_extractor.cpp:128](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L128) |
| `/raw_obstacles` | **`obstacles_pub_`** [obstacle_extractor.cpp:141](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L141) | **`obstacles_sub_`** [obstacle_tracker.cpp:161](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L161) |
| `/odom` | DLIO (HW) / 브리지 (SIM) | **`odom_sub_`** [obstacle_tracker.cpp:157](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L157) |
| `/tracked_obstacles` | **`obstacles_pub_`** [obstacle_tracker.cpp:164](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L164) | **`sub_`** [safety_obstacle_filter_node.cpp:36](../ros2/src/g1_perception/safety_obstacle_filter/src/safety_obstacle_filter_node.cpp#L36) |
| **`/obstacles_safe`** | **`pub_`** [safety_obstacle_filter_node.cpp:34](../ros2/src/g1_perception/safety_obstacle_filter/src/safety_obstacle_filter_node.cpp#L34) | **`impl_->sub`** [obstacle_source.cpp:300](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L300) |
| `/dpcbf/status` | `impl_->diag_pub` [obstacle_source.cpp:295](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L295) | (진단 소비자) |

**제어가 실제로 장애물을 받는 짝은 마지막에서 두 번째 줄**이다.

## C-2. 그 짝의 실제 코드

**보내는 쪽** — [safety_obstacle_filter_node.cpp:34-45](../ros2/src/g1_perception/safety_obstacle_filter/src/safety_obstacle_filter_node.cpp#L34-L45)

```cpp
pub_ = create_publisher<obstacle_detector::msg::Obstacles>(
    "obstacles_safe", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
sub_ = create_subscription<obstacle_detector::msg::Obstacles>(
    "tracked_obstacles", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
    // ConstSharedPtr, not `const Msg&`: rclcpp only accepts const-reference
    // subscription callbacks from Galactic on, and the G1's onboard
    // computer runs Foxy.
    [this](obstacle_detector::msg::Obstacles::ConstSharedPtr msg) {
      pub_->publish(Apply(*msg, params_, now().seconds(), &stats_));
      ReportStats();
    });
```

**받는 쪽** — [obstacle_source.cpp:298-311](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L298-L311)

```cpp
if (cfg.mode != Mode::kOracle) {
  // /obstacles_safe is Reliable depth 1 — latest wins (§7.1).
  impl_->sub =
      impl_->node->create_subscription<obstacle_detector::msg::Obstacles>(
          cfg.topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
          [impl = impl_.get()](
              obstacle_detector::msg::Obstacles::ConstSharedPtr m) {
            impl->OnObstacles(*m);
          });
}
```

**받아서 변수에 담는 곳** — [obstacle_source.cpp:82-102](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L82-L102)

```cpp
void OnObstacles(const obstacle_detector::msg::Obstacles& msg) {
  ObstacleFrame frame;
  frame.stamp = StampToSec(msg.header.stamp);
  std::size_t n = 0;
  for (const auto& c : msg.circles) {
    if (n >= kMaxObstacles) { ...; break; }
    auto& o = frame.obstacles[n++];
    o.x          = c.center.x;
    o.y          = c.center.y;
    o.radius     = c.radius;
    o.velocity_x = c.velocity.x;
    o.velocity_y = c.velocity.y;
    o.id         = UidToId(c.uid);
  }
  frame.count = n;
  buffer.Publish(frame);
}
```

## C-3. 변수 이름 대응표

| 개념 | ROS 메시지 필드 | 어댑터 내부 | 제어 루프 |
|---|---|---|---|
| 위치 x | `c.center.x` | `o.x` | `snap.obstacles[i].x` |
| 위치 y | `c.center.y` | `o.y` | `snap.obstacles[i].y` |
| 반경 | `c.radius` | `o.radius` | `snap.obstacles[i].radius` |
| 속도 x | `c.velocity.x` | `o.velocity_x` | `snap.obstacles[i].velocity_x` |
| 속도 y | `c.velocity.y` | `o.velocity_y` | `snap.obstacles[i].velocity_y` |
| ID | `c.uid` (uint64) | `o.id` (int) | `snap.obstacles[i].id` |
| 시각 | `msg.header.stamp` | `frame.stamp` (double) | `snap.age_s` |
| 개수 | `msg.circles.size()` | `frame.count` | `snap.obstacles.size()` |

컨테이너 이름:

| 위치 | 변수 | 타입 |
|---|---|---|
| 수신 콜백 | `frame` | `ObstacleFrame` (지역변수) |
| 버퍼 | `impl_->buffer` | `ObstacleBuffer` |
| 제어 스레드 재사용 버퍼 | `impl_->scratch_frame` | `ObstacleFrame` |
| 외삽 결과 | `impl_->scratch_snapshot` | `MaterializedSnapshot` |
| **최종** | **`snap.obstacles`** | **`std::vector<dpcbf::ObstacleState>`** |

## C-4. 주의 — 기본 모드에서는 이 섭스크라이버가 안 만들어진다

[obstacle_source.cpp:298](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L298)의 `if (cfg.mode != Mode::kOracle)`.

| `UNITREE_DPCBF_MODE` | `impl_->sub` | `snap.obstacles`의 출처 |
|---|---|---|
| `oracle` (**기본값**) | 생성 안 됨 | `oracle_provider` 람다 [main.cc:895-907](../simulate/src/main.cc#L895-L907) — 시뮬 정답값 |
| `shadow` | 생성됨 | `oracle_provider` (제어) + 추정값은 오차 통계에만 |
| `estimated` | 생성됨 | `/obstacles_safe` |

---

# Part D. 질문 2 직답 — 추정 과정 핵심 코드 위치

## D-1. 우선순위 순 목록

| 순위 | 무엇 | 위치 |
|---|---|---|
| ★★★ | **데이터 연관 본체** | [obstacle_tracker.cpp:268-388](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L268-L388) `obstaclesCallbackCircles` |
| ★★★ | **KF 예측/보정** | [tracked_circle_obstacle.h:56-111](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L56-L111) |
| ★★★ | **선분 → 원 피팅** | [obstacle_extractor.cpp:397-444](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L397-L444) + [circle.h:53-57](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/circle.h#L53-L57) |
| ★★ | 안전 반경 팽창 | [gating.h:87-135](../ros2/src/g1_perception/safety_obstacle_filter/include/safety_obstacle_filter/gating.h#L87-L135) `Apply` |
| ★★ | 시간 외삽 + staleness | [obstacle_buffer.h:89-124](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L89-L124) `Materialize` |
| ★★ | KF 수식 | [kalman.h:64-76](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/kalman.h#L64-L76) |
| ★ | 점 그룹화 | [obstacle_extractor.cpp:236-285](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L236-L285) `groupPoints` |
| ★ | 연관 비용 함수 | [obstacle_tracker.cpp:505-533](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L505-L533) |
| ★ | 프레임 변환 (→odom) | [obstacle_extractor.cpp:586-620](../ros2/src/external/obstacle_detector_2/src/obstacle_extractor.cpp#L586-L620) |
| ★ | staleness 사다리 | [obstacle_buffer.h:31-64](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L31-L64) |

## D-2. "추정"이 실제로 일어나는 곳은 3군데뿐

나머지는 전부 자료형 변환·게이팅·복사다.

### ① 형상 추정 — 선분을 원으로

[circle.h:53-57](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/circle.h#L53-L57)

```cpp
/*
 * Create a circle by taking the segment as a base of equilateral
 * triangle. The circle is circumscribed on this triangle.
 */
Circle(const Segment& s) {
  radius = 0.5773502 * s.length();  // sqrt(3)/3 * length
  center = (s.first_point + s.last_point - radius * s.normal()) / 2.0;
  point_sets = s.point_sets;
}
```

LiDAR는 물체의 **앞면 원호**만 본다. 그 원호를 밑변으로 하는 정삼각형의 외접원을 물체로 가정한다.
원호가 짧을수록(= 멀거나 가려짐) 반경이 과소추정되고, 그래서 `radius_enlargement`가 붙는다.

### ② 상태 추정 — 칼만필터

축마다 상태 2차원 `[값, 변화율]`, 관측 1차원. x/y/r 세 축이 **독립**이다.

[tracked_circle_obstacle.h:50](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L50)

```cpp
: obstacle_(obstacle), kf_x_(0, 1, 2), kf_y_(0, 1, 2), kf_r_(0, 1, 2) {
//                            ↑  ↑  ↑
//                     dim_in ┘  │  └ dim_state = 2 : [위치, 속도]
//                        dim_out ┘   = 1 : 위치만 관측
```

[kalman.h:64-76](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/kalman.h#L64-L76)

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

행렬 구성 — [tracked_circle_obstacle.h:162-180](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L162-L182)

```
A = [1  dt]     C = [1  0]      Q = [process_variance      0            ]
    [0   1]                         [0                process_rate_var  ]

R = [measurement_variance]      P(0,0) = R,  P(1,1) = 2R/dt²   (2점 초기화)
```

### ③ 시간 추정 — 10 Hz를 1 kHz로

[obstacle_buffer.h:111-118](../ros2/src/g1_perception/dpcbf_ros_adapter/include/dpcbf_ros_adapter/obstacle_buffer.h#L111-L118)

```cpp
for (std::size_t i = 0; i < frame.count; ++i) {
  dpcbf::ObstacleState o = frame.obstacles[i];
  o.x += o.velocity_x * dt_extrap;
  o.y += o.velocity_y * dt_extrap;
  if (inflate_horizon > 0.0) {
    o.radius += std::hypot(o.velocity_x, o.velocity_y) * inflate_horizon;
  }
  out->obstacles.push_back(o);
}
```

속도가 ②에서 나왔기 때문에 이게 가능하다. 추출기 단계에서 끊으면 외삽할 수 없다.

## D-3. 세 추정이 서로 어떻게 의존하는가

```
  ①형상 추정 ────▶ radius            ────┐
       │                                  ├──▶ ②KF ────▶ velocity ────▶ ③시간 외삽
       └────────▶ center                ──┘        │                        │
                                                    └──▶ P(0,0) ──▶ σ ──▶ 반경 팽창
                                                                        (기본 OFF)
```

- ①의 중심 바이어스가 ②의 관측 잡음으로 들어간다 → `fixed_inflation`이 이걸 덮는다
- ②의 속도가 없으면 ③의 외삽도, `|v|·latency_horizon` 팽창도 불가능하다
- ②의 `P(0,0)`이 σ 항의 입력이지만, `measurement_variance`가 미보정이라 현재 OFF

---

# 부록. 한 줄 요약

- **받아오는 코드** → [obstacle_source.cpp:82-102](../ros2/src/g1_perception/dpcbf_ros_adapter/src/obstacle_source.cpp#L82-L102) `OnObstacles()`, 변수는 `frame.obstacles[n]` → `snap.obstacles`
- **추정 본체** → [obstacle_tracker.cpp:268-388](../ros2/src/external/obstacle_detector_2/src/obstacle_tracker.cpp#L268-L388) + [tracked_circle_obstacle.h:56-111](../ros2/src/external/obstacle_detector_2/include/obstacle_detector/utilities/tracked_circle_obstacle.h#L56-L111)
- **파일 관계의 원리** → 규칙은 ROS-free 헤더, 배선은 .cpp. 토픽 배선은 [perception.launch.py](../ros2/src/g1_perception/g1_perception_bringup/launch/perception.launch.py) 한 곳
- **변수 관계의 핵심** → `velocity`는 KF가 만들고, `radius`는 5번 바뀌고, `header.stamp`는 측정 시각으로 끝까지 전파된다
