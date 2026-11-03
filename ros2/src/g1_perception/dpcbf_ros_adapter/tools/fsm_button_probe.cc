// FSM button probe (workstream B): does a ScriptedJoystick chord actually
// reach g1_ctrl's transition predicate?
//
// "An unreceived button and a rejected transition look identical from the
// outside" — this tool separates them. It runs the EXACT code path the FSM
// runs, and nothing else:
//
//   * unitree::robot::g1::subscription::LowState  — the same subscription
//     class FSMState::lowstate is, so wireless_remote is decoded by the same
//     update() (including the receiver-side Axis low-pass on LT/RT).
//   * unitree::common::dsl::{Parser,Compile} from deploy/include — the same
//     compiler FSMState's constructor uses, fed the same condition strings
//     out of deploy/robots/g1/config/config.yaml.
//   * update() polled at 1 kHz, matching CtrlFSM's dt (0.001) / pre_run().
//
// It never publishes and never touches lowcmd: it is a passive observer that
// can be run alongside or instead of g1_ctrl.
//
// Usage: fsm_button_probe <deploy_config.yaml> <seconds> [trace.csv]
// Exit 0 if every transition predicate in the config fired at least once,
// 1 otherwise — so it is usable as a gate.

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/dds_wrapper/robots/g1/g1.h>

#include "unitree_joystick_dsl.hpp"

namespace {

struct Watch {
  std::string from;
  std::string to;
  std::string condition;
  std::function<bool(const unitree::common::UnitreeJoystick&)> pred;
  long fired = 0;
  double first_fire = -1.0;
};

// Every atom name the DSL can resolve, so the trace can show the operand that
// was false when a chord did not fire.
const char* kTraced[] = {"LT", "RT", "up", "down", "A", "B", "X", "Y",
                         "LB", "RB", "start", "back"};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: fsm_button_probe <deploy_config.yaml> <seconds> "
                 "[trace.csv]\n";
    return 2;
  }
  const std::string cfg_path = argv[1];
  const double secs = std::stod(argv[2]);
  const std::string trace_path = argc > 3 ? argv[3] : "";

  YAML::Node cfg;
  try {
    cfg = YAML::LoadFile(cfg_path);
  } catch (const std::exception& e) {
    std::cerr << "cannot load " << cfg_path << ": " << e.what() << "\n";
    return 2;
  }

  std::vector<Watch> watches;
  auto fsms = cfg["FSM"]["_"];
  for (auto it = fsms.begin(); it != fsms.end(); ++it) {
    const std::string from = it->first.as<std::string>();
    auto trans = cfg["FSM"][from]["transitions"];
    if (!trans) continue;
    for (auto t = trans.begin(); t != trans.end(); ++t) {
      Watch w;
      w.from = from;
      w.to = t->first.as<std::string>();
      w.condition = t->second.as<std::string>();
      try {
        unitree::common::dsl::Parser p(w.condition);
        w.pred = unitree::common::dsl::Compile(*p.Parse());
      } catch (const std::exception& e) {
        std::cerr << "cannot compile '" << w.condition << "': " << e.what()
                  << "\n";
        return 2;
      }
      watches.push_back(std::move(w));
    }
  }
  std::cout << "probe: " << watches.size() << " transition predicates from "
            << cfg_path << "\n";

  unitree::robot::ChannelFactory::Instance()->Init(0, "lo");
  auto lowstate = std::make_shared<unitree::robot::g1::subscription::LowState>();
  lowstate->wait_for_connection();
  std::cout << "probe: rt/lowstate connected\n";

  std::ofstream trace;
  if (!trace_path.empty()) {
    trace.open(trace_path);
    trace << "t,tick";
    for (const char* n : kTraced) trace << "," << n << "_p," << n << "_op";
    trace << ",LT_val,RT_val";
    for (const auto& w : watches) trace << "," << w.from << "2" << w.to;
    trace << "\n";
  }

  // Peak analog value the receiver-side Axis reaches: the whole question of
  // whether a chord's *axis* half ever crosses Axis::threshold (0.5).
  float lt_peak = 0.f, rt_peak = 0.f;
  std::map<std::string, long> pressed_ticks, onpressed_ticks;

  const auto t0 = std::chrono::steady_clock::now();
  auto next = t0;
  long ticks = 0;
  while (true) {
    const double t =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
    if (t >= secs) break;

    lowstate->update();  // exactly what FSMState::pre_run() does
    auto& j = lowstate->joystick;
    lt_peak = std::max(lt_peak, j.LT());
    rt_peak = std::max(rt_peak, j.RT());
    for (const char* n : kTraced) {
      const auto& kb = unitree::common::dsl::GetKey(j, n);
      if (kb.pressed) pressed_ticks[n]++;
      if (kb.on_pressed) onpressed_ticks[n]++;
    }
    for (auto& w : watches) {
      if (w.pred(j)) {
        if (w.fired == 0) w.first_fire = t;
        w.fired++;
      }
    }
    if (trace.is_open()) {
      trace << t << "," << lowstate->msg_.tick();
      for (const char* n : kTraced) {
        const auto& kb = unitree::common::dsl::GetKey(j, n);
        trace << "," << (kb.pressed ? 1 : 0) << "," << (kb.on_pressed ? 1 : 0);
      }
      trace << "," << j.LT() << "," << j.RT();
      for (const auto& w : watches) trace << "," << (w.pred(j) ? 1 : 0);
      trace << "\n";
    }
    ticks++;
    next += std::chrono::milliseconds(1);
    std::this_thread::sleep_until(next);
  }

  std::cout << "\n--- " << ticks << " ticks over " << secs << " s ---\n";
  std::cout << "receiver-side axis peaks (Axis::threshold = 0.5): "
            << "LT " << lt_peak << "  RT " << rt_peak << "\n";
  std::cout << "\nkey            pressed_ticks  on_pressed_edges\n";
  for (const char* n : kTraced) {
    printf("%-12s   %13ld  %16ld\n", n, pressed_ticks[n], onpressed_ticks[n]);
  }
  std::cout << "\ntransition predicates:\n";
  int missed = 0;
  for (const auto& w : watches) {
    printf("  %-24s <- %-24s  fired %6ld ticks", (w.from + " -> " + w.to).c_str(),
           w.condition.c_str(), w.fired);
    if (w.fired) {
      printf("  first @ %.3f s\n", w.first_fire);
    } else {
      printf("  NEVER\n");
      missed++;
    }
  }
  return missed ? 1 : 0;
}
