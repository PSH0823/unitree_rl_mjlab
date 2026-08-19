#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"
#include <set>

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        // Communication loss always wins over operator requests.
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );

        auto transitions = param::config["FSM"][state_string]["transitions"];
        const auto console = param::config["console_fsm_control"];
        const bool console_enabled = console &&
            console["enabled"].as<bool>(false) &&
            (!console["simulation_only"].as<bool>(true) ||
             param::is_simulation);
        std::set<int> console_targets;

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );

                if (console_enabled && fsm_id >= 0 && fsm_id <= 9) {
                    console_targets.insert(fsm_id);
                    const std::string id_key = std::to_string(fsm_id);
                    registered_checks.emplace_back(std::make_pair(
                        [id_key]()->bool {
                            return FSMState::keyboard &&
                                   FSMState::keyboard->on_pressed &&
                                   FSMState::keyboard->key() == id_key;
                        },
                        fsm_id));
                }
            }
        }
        if (console_enabled) {
            registered_checks.emplace_back(std::make_pair(
                [state_string, console_targets]()->bool {
                    if (!FSMState::keyboard || !FSMState::keyboard->on_pressed) {
                        return false;
                    }
                    const std::string key = FSMState::keyboard->key();
                    if (key.size() != 1 || key[0] < '0' || key[0] > '9') {
                        return false;
                    }
                    const int requested = key[0] - '0';
                    if (!console_targets.count(requested)) {
                        spdlog::warn(
                            "FSM numeric request rejected: {} -> id {} is not a configured transition",
                            state_string, requested);
                    }
                    return false;
                }, 0));
        }
    }

    void pre_run()
    {
        lowstate->update();
        if(keyboard) keyboard->update();
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
};
