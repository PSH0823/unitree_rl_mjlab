#pragma once

#include <algorithm>
#include <cmath>

#include "isaaclab/envs/manager_based_rl_env.h"

namespace isaaclab
{
namespace mdp
{

inline bool bad_orientation(ManagerBasedRLEnv* env, float limit_angle = 1.0)
{
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    const float down_projection = std::clamp(-data[2], -1.0f, 1.0f);
    return std::fabs(std::acos(down_projection)) > limit_angle;
}

}
} 
