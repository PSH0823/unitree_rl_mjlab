// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <unitree/common/thread/recurrent_thread.hpp>
#include "BaseState.h"
#include "param.h"
#include <algorithm>
#include <spdlog/spdlog.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

class CtrlFSM
{
public:
    CtrlFSM(std::shared_ptr<BaseState> initstate)
    {
        // Initialize FSM states
        states.push_back(std::move(initstate));

    }

    CtrlFSM(YAML::Node cfg)
    {
        auto fsms = cfg["_"]; // enabled FSMs

        // register FSM string map; used for state transition
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            FSMStringMap.insert({id, fsm_name});
        }

        // Initialize FSM states
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            std::string fsm_type = it->second["type"] ? it->second["type"].as<std::string>() : fsm_name;
            auto fsm_class = getFsmMap().find("State_" + fsm_type);
            if (fsm_class == getFsmMap().end()) {
                throw std::runtime_error("FSM: Unknown FSM type " + fsm_type);
            }
            auto state_instance = fsm_class->second(id, fsm_name);
            add(state_instance);
        }
    }

    void start() 
    {
        // Start From State_Passive
        currentState = states[0];
        currentState->enter();

        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "FSM", 0, this->dt * 1e6, &CtrlFSM::run_, this);
        spdlog::info("FSM: Start {}", currentState->getStateString());
        if (param::config["console_fsm_control"] &&
            param::config["console_fsm_control"]["enabled"].as<bool>(false) &&
            (!param::config["console_fsm_control"]["simulation_only"].as<bool>(true) ||
             param::is_simulation)) {
            spdlog::info("FSM numeric control enabled (configured transitions only)");
            for (const auto& entry : FSMStringMap.left) {
                spdlog::info("  {}: {}", entry.first, entry.second);
            }
        }
    }

    void add(std::shared_ptr<BaseState> state)
    {
        for(auto & s : states)
        {
            if(s->isState(state->getState()))
            {
                spdlog::error("FSM: State_{} already exists", state->getStateString());
                std::exit(0);
            }
        }

        states.push_back(std::move(state));
    }
    
    ~CtrlFSM()
    {
        shutdown();
    }

    void shutdown()
    {
        if (shutdown_) return;
        shutdown_ = true;

        // Stop the recurrent callback before touching currentState.  This is
        // also important for Navigation, whose exit() joins its policy thread.
        fsm_thread_.reset();

        if (currentState) {
            currentState->exit();

            // Leave a damping/passive command on the Unitree channel instead
            // of exiting with the last policy torque/position target latched.
            auto passive = std::find_if(
                states.begin(), states.end(), [](const auto& state) {
                    return state->getStateString() == "Passive";
                });
            if (passive != states.end()) {
                currentState = *passive;
                currentState->enter();
                spdlog::info("FSM: Sending Passive command before shutdown");
                for (int i = 0; i < 100; ++i) {
                    currentState->pre_run();
                    currentState->run();
                    currentState->post_run();
                    usleep(1000);
                }
                currentState->exit();
            }
            currentState.reset();
        }
        states.clear();
    }

    std::vector<std::shared_ptr<BaseState>> states;
private:
    const double dt = 0.001;

    void run_()
    {
        currentState->pre_run();
        currentState->run();
        currentState->post_run();
        
        // Check if need to change state
        int nextStateMode = 0;
        for(int i(0); i<currentState->registered_checks.size(); i++)
        {
            if(currentState->registered_checks[i].first())
            {
                nextStateMode = currentState->registered_checks[i].second;
                break;
            }
        }

        if(nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            for(auto & state : states)
            {
                if(state->isState(nextStateMode))
                {
                    spdlog::info("FSM: Change state from {} to {}", currentState->getStateString(), state->getStateString());
                    currentState->exit();
                    currentState = state;
                    currentState->enter();
                    break;
                }
            }
        }
    }

    std::shared_ptr<BaseState> currentState;
    unitree::common::RecurrentThreadPtr fsm_thread_;
    bool shutdown_ = false;
};
