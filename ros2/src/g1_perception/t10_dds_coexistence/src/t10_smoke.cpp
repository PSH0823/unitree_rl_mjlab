// T10 DDS coexistence smoke (§16.2, R-3).
//
// One process, both middleware stacks, one CycloneDDS domain:
//   SDK2 side : ChannelPublisher + ChannelSubscriber on "rt/t10_smoke"
//               (+ passive ChannelSubscriber on "rt/lowstate" — counts
//               messages when the simulator is running)
//   ROS2 side : rclcpp publisher + subscription on "/t10_ping"
//
// PASS = both loopbacks deliver messages, no crash, and the process maps
// exactly the CycloneDDS libraries we expect (printed for the log).
// Run with RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; --require-lowstate makes
// live-simulator reception mandatory.
#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <std_msgs/msg/u_int64.hpp>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using unitree_hg::msg::dds_::LowState_;

std::atomic<uint64_t> sdk2_self_rx{0};
std::atomic<uint64_t> sdk2_lowstate_rx{0};
std::atomic<uint64_t> ros2_rx{0};

static void PrintLoadedDdsLibs() {
  std::ifstream maps("/proc/self/maps");
  std::string line, last;
  std::cout << "[t10] loaded DDS/rmw libraries:" << std::endl;
  while (std::getline(maps, line)) {
    const auto pos = line.find('/');
    if (pos == std::string::npos) continue;
    const std::string path = line.substr(pos);
    if (path == last) continue;
    if (path.find("ddsc") != std::string::npos ||
        path.find("rmw_") != std::string::npos ||
        path.find("cyclonedds") != std::string::npos) {
      std::cout << "  " << path << std::endl;
      last = path;
    }
  }
}

int main(int argc, char** argv) {
  int duration_s = 10;
  int domain = 0;
  // Empty = JOIN the rmw-created domain (the only order that works in one
  // process, see below); the interface comes from the shared CYCLONEDDS_URI.
  std::string interface;
  bool require_lowstate = false;
  bool sdk2_first = false;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--duration") && i + 1 < argc) duration_s = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--domain") && i + 1 < argc) domain = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--interface") && i + 1 < argc) interface = argv[++i];
    else if (!std::strcmp(argv[i], "--require-lowstate")) require_lowstate = true;
    else if (!std::strcmp(argv[i], "--sdk2-first")) sdk2_first = true;
  }

  // Init order is load-bearing (R-3): rmw_cyclonedds calls dds_create_domain
  // and hard-fails with PRECONDITION_NOT_MET if the domain already exists in
  // this process — which it does whenever SDK2's ChannelFactory ran first.
  // ROS2 must therefore initialize BEFORE the SDK2 factory; the SDK2
  // participant then joins the existing domain. --sdk2-first reproduces the
  // failure mode for documentation.
  if (sdk2_first) {
    std::cout << "[t10] init order: SDK2 -> ROS2 (expected to fail)" << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(domain, interface);
    rclcpp::init(argc, argv);
  } else {
    std::cout << "[t10] init order: ROS2 -> SDK2" << std::endl;
    rclcpp::init(argc, argv);
  }

  auto node = rclcpp::Node::make_shared("t10_smoke");
  auto ping_pub = node->create_publisher<std_msgs::msg::UInt64>("/t10_ping", 10);
  auto ping_sub = node->create_subscription<std_msgs::msg::UInt64>(
      "/t10_ping", 10, [](std_msgs::msg::UInt64::ConstSharedPtr) { ros2_rx++; });

  // ---- SDK2 stack (same process) ----
  if (!sdk2_first) {
    unitree::robot::ChannelFactory::Instance()->Init(domain, interface);
  }

  unitree::robot::ChannelSubscriber<LowState_> self_sub("rt/t10_smoke");
  self_sub.InitChannel([](const void*) { sdk2_self_rx++; }, 1);

  unitree::robot::ChannelSubscriber<LowState_> lowstate_sub("rt/lowstate");
  lowstate_sub.InitChannel([](const void*) { sdk2_lowstate_rx++; }, 1);

  unitree::robot::ChannelPublisher<LowState_> self_pub("rt/t10_smoke");
  self_pub.InitChannel();

  std::cout << "[t10] rmw implementation: " << rmw_get_implementation_identifier()
            << " | domain " << domain << " | interface " << interface << std::endl;

  LowState_ sdk2_msg{};
  std_msgs::msg::UInt64 ros2_msg;
  const auto t_end = std::chrono::steady_clock::now() + std::chrono::seconds(duration_s);
  uint64_t tick = 0;
  while (std::chrono::steady_clock::now() < t_end) {
    sdk2_msg.tick() = tick;
    self_pub.Write(sdk2_msg);
    ros2_msg.data = tick++;
    ping_pub->publish(ros2_msg);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  PrintLoadedDdsLibs();
  std::cout << "[t10] sdk2 self-loopback rx: " << sdk2_self_rx.load() << std::endl;
  std::cout << "[t10] sdk2 rt/lowstate  rx: " << sdk2_lowstate_rx.load()
            << (require_lowstate ? " (required)" : " (informational)") << std::endl;
  std::cout << "[t10] ros2 /t10_ping    rx: " << ros2_rx.load() << std::endl;

  const bool cyclone = std::string(rmw_get_implementation_identifier())
                           .find("cyclonedds") != std::string::npos;
  bool pass = sdk2_self_rx > 0 && ros2_rx > 0 && cyclone;
  if (require_lowstate) pass = pass && sdk2_lowstate_rx > 0;

  rclcpp::shutdown();
  std::cout << (pass ? "[t10] PASS" : "[t10] FAIL") << std::endl;
  return pass ? 0 : 1;
}
