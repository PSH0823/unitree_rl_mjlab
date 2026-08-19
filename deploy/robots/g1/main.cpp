#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Navigation.h"
#include "State_Mimic.h"
#include <rclcpp/rclcpp.hpp>

#include <csignal>
#include <cstdlib>
#include <string>

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

namespace {
volatile std::sig_atomic_t shutdown_requested = 0;

void request_shutdown(int)
{
    shutdown_requested = 1;
}
}  // namespace

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        spdlog::critical("The other process is using the lowcmd channel, please close it first.");
        unitree::robot::go2::shutdown();
        // exit(0);
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);
    const std::string network = vm["network"].as<std::string>();
    param::is_simulation = network == "lo";

    // rmw_cyclonedds and unitree_sdk2 share CycloneDDS in this process.
    // The ROS participant must create the domain first; ChannelFactory then
    // joins it by using an empty interface argument.  Creating only the
    // rclcpp context is insufficient because the DDS domain is created when
    // the first node is constructed (see the T10 coexistence smoke test).
    const char* rmw = std::getenv("RMW_IMPLEMENTATION");
    const bool shared_cyclone =
        rmw && std::string(rmw).find("cyclonedds") != std::string::npos;
    if (shared_cyclone && !std::getenv("CYCLONEDDS_URI")) {
        const std::string uri =
            "<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"" +
            network + (network == "lo" ? "\" multicast=\"true" : "") +
            "\"/></Interfaces></General><Discovery><ParticipantIndex>auto"
            "</ParticipantIndex><MaxAutoParticipantIndex>120"
            "</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>";
        setenv("CYCLONEDDS_URI", uri.c_str(), 0);
    }
    if (!std::getenv("ROS_DOMAIN_ID")) {
        setenv("ROS_DOMAIN_ID", "0", 0);
    }
    rclcpp::init(
        argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, request_shutdown);
    std::signal(SIGTERM, request_shutdown);
    auto ros_bootstrap_node =
        rclcpp::Node::make_shared("g1_controller_dds_bootstrap");

    std::cout << " --- Unitree Robotics --- \n";
    std::cout << "     G1-29dof Controller \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(
        0, shared_cyclone ? "" : network);

    init_fsm_state();

    FSMState::lowcmd->msg_.mode_machine() = 5; // 29dof
    if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
        spdlog::critical("Unmatched robot type.");
        exit(-1);
    }

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
    std::cout << "And then press [R2 + A] to start controlling the robot.\n";
    std::cout << "And then press [R1 + A/B/Y/X] to control the robot dance.\n";

    while (!shutdown_requested)
    {
        usleep(100000);
    }

    spdlog::info("Shutdown requested");
    fsm->shutdown();
    fsm.reset();
    ros_bootstrap_node.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
}
