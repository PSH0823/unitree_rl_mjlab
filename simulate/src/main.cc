// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// !!! hack code: make glfw_adapter.window_ public
#define private public
#include "glfw_adapter.h"
#undef private

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "simulate.h"
#include "array_safety.h"
#include "unitree_sdk2_bridge.h"
#include "param.h"
#include "dpcbf/dynamic_obstacles.h"
#include "dpcbf/dpcbf_safety_filter.h"
#include "dpcbf/dpcbf_visualizer.h"
// Phase-4 seam (§10): the axes<->command mapping moved verbatim into the
// adapter package so the T1 replay harness compiles the exact same code;
// filter_io_log is the H-10 capture instrument (env UNITREE_DPCBF_FILTER_LOG).
// Both headers are rclcpp-free; ObstacleSource itself links rclcpp and is
// only used in the ROS2 build.
#include "dpcbf_ros_adapter/dpcbf_seam.h"
#include "dpcbf_ros_adapter/filter_io_log.h"
// Header is rclcpp-free; the OFF build uses only the Mode enum (no
// ObstacleSource is constructed, so nothing links against the adapter lib).
#include "dpcbf_ros_adapter/obstacle_source.h"
#ifdef UNITREE_MUJOCO_WITH_ROS2
#include "ros2_bridge.h"
#endif

#include <fstream>
#include <sstream>
#include <vector>

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"
#define NUM_MOTOR_IDL_GO 20

extern "C"
{
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

class ElasticBand
{
public:
  ElasticBand(){};
  void Advance(std::vector<double> x, std::vector<double> dx)
  {
    std::vector<double> delta_x = {0.0, 0.0, 0.0};
    delta_x[0] = point_[0] - x[0];
    delta_x[1] = point_[1] - x[1];
    delta_x[2] = point_[2] - x[2];
    double distance = sqrt(delta_x[0] * delta_x[0] + delta_x[1] * delta_x[1] + delta_x[2] * delta_x[2]);

    std::vector<double> direction = {0.0, 0.0, 0.0};
    direction[0] = delta_x[0] / distance;
    direction[1] = delta_x[1] / distance;
    direction[2] = delta_x[2] / distance;

    double v = dx[0] * direction[0] + dx[1] * direction[1] + dx[2] * direction[2];

    f_[0] = (stiffness_ * (distance - length_) - damping_ * v) * direction[0];
    f_[1] = (stiffness_ * (distance - length_) - damping_ * v) * direction[1];
    f_[2] = (stiffness_ * (distance - length_) - damping_ * v) * direction[2];
  }


  // Scripted lowering (workstream B). Interactively the band is a hoist you
  // toggle with key 9, which is why every closed-loop session so far ran with
  // the robot suspended — the ±0.45 m swing and free yaw the Phase-4 shadow
  // deltas had to be attributed to. A headless walking run needs the OTHER
  // half of the operator procedure: hold the robot while g1_ctrl's FixStand
  // loads the legs, then LOWER it and let it stand on its own.
  //
  // Gain(t) is 1 until release_t_, ramps linearly to 0 over release_ramp_ and
  // stays 0 — the sim analogue of paying out the strap, not of cutting it.
  // Both are sim-time, set only by env UNITREE_MUJOCO_BAND_RELEASE="t0[,ramp]"
  // and UNITREE_MUJOCO_BAND_LENGTH; unset leaves the stock hoist untouched.
  double Gain(double t) const
  {
    if (release_t_ < 0.0) return 1.0;               // never released (stock)
    if (t <= release_t_) return 1.0;
    if (release_ramp_ <= 0.0) return 0.0;
    const double g = 1.0 - (t - release_t_) / release_ramp_;
    return g > 0.0 ? g : 0.0;
  }
  bool Released(double t) const
  {
    return release_t_ >= 0.0 && t > release_t_ + release_ramp_;
  }

  double stiffness_ = 200;
  double damping_ = 100;
  std::vector<double> point_ = {0, 0, 3};
  double length_ = 0.0;
  bool enable_ = true;
  double release_t_ = -1.0;    // <0: never (stock behaviour)
  double release_ramp_ = 1.0;  // s
  std::vector<double> f_ = {0, 0, 0};
};
inline ElasticBand elastic_band;


namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;

  dpcbf::DynamicObstacleManager dynamic_obstacles;
  dpcbf::DpcbfSafetyFilter safety_filter;
  dpcbf::DpcbfVisualizer dpcbf_visualizer;

  // control noise variables
  mjtNum *ctrlnoise = nullptr;

  using Seconds = std::chrono::duration<double>;

  // AxisToCommand / CommandToAxis moved verbatim to
  // dpcbf_ros_adapter/axis_command_map.h (shared with the T1 replay harness).

  dpcbf::RobotState ReadRobotGroundTruth(const mjModel* model, const mjData* data,
                                         int body_id)
  {
    dpcbf::RobotState state;
    if (!model || !data || body_id < 0)
    {
      return state;
    }
    const mjtNum* position = data->xpos + 3 * body_id;
    const mjtNum* rotation = data->xmat + 9 * body_id;
    state.x = position[0];
    state.y = position[1];
    state.phi = std::atan2(rotation[3], rotation[0]);
    mjtNum body_velocity[6] = {};
    mj_objectVelocity(model, data, mjOBJ_BODY, body_id, body_velocity, 1);
    state.sagittal_velocity = body_velocity[3];
    state.lateral_velocity = body_velocity[4];
    return state;
  }

  //---------------------------------------- plugin handling -----------------------------------------

  // return the path to the directory containing the current executable
  // used to determine the location of auto-loaded plugin libraries
  std::string getExecutableDir()
  {
#if defined(_WIN32) || defined(__CYGWIN__)
    constexpr char kPathSep = '\\';
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      DWORD buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
        if (written < buf_size)
        {
          success = true;
        }
        else if (written == buf_size)
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
        else
        {
          std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
          return "";
        }
      }
      return realpath.get();
    }();
#else
    constexpr char kPathSep = '/';
#if defined(__APPLE__)
    std::unique_ptr<char[]> buf(nullptr);
    {
      std::uint32_t buf_size = 0;
      _NSGetExecutablePath(nullptr, &buf_size);
      buf.reset(new char[buf_size]);
      if (!buf)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }
      if (_NSGetExecutablePath(buf.get(), &buf_size))
      {
        std::cerr << "unexpected error from _NSGetExecutablePath\n";
      }
    }
    const char *path = buf.get();
#else
    const char *path = "/proc/self/exe";
#endif
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      std::uint32_t buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        std::size_t written = readlink(path, realpath.get(), buf_size);
        if (written < buf_size)
        {
          realpath.get()[written] = '\0';
          success = true;
        }
        else if (written == -1)
        {
          if (errno == EINVAL)
          {
            // path is already not a symlink, just use it
            return path;
          }

          std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
          return "";
        }
        else
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
      }
      return realpath.get();
    }();
#endif

    if (realpath.empty())
    {
      return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
      if (realpath.c_str()[i] == kPathSep)
      {
        return realpath.substr(0, i);
      }
    }

    // don't scan through the entire file system's root
    return "";
  }

  // scan for libraries in the plugin directory to load additional plugins
  void scanPluginLibraries()
  {
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
      std::printf("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i)
      {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    }

    // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
    const std::string sep = "\\";
#else
    const std::string sep = "/";
#endif

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
      return;
    }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
  }

  //------------------------------------------- simulation -------------------------------------------

  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mjSpec* spec = mj_parseXML(filename, nullptr, loadError, kErrorLength);
      if (spec)
      {
        try
        {
          dynamic_obstacles.AddToSpec(spec);
          mnew = mj_compile(spec, nullptr);
          if (!mnew)
          {
            mju::strcpy_arr(loadError, mjs_getError(spec));
          }
#ifdef UNITREE_MUJOCO_WITH_ROS2
          // The sidecar mirrors the COMPILED model (incl. runtime-added
          // obstacle mocap bodies), not the raw scene file (§11.2).
          if (mnew)
          {
            Ros2DumpMirrorModel(spec);
          }
#endif
        }
        catch (const std::exception& error)
        {
          mju::strcpy_arr(loadError, error.what());
        }
        mj_deleteSpec(spec);
      }
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    return mnew;
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim)
  {
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    // ChannelFactory::Instance()->Init(0);
    // UnitreeDds ud(d);

    // run until asked to exit
    while (!sim.exitrequest.load())
    {
      if (sim.droploadrequest.load())
      {
        sim.LoadMessage(sim.dropfilename);
        mjModel *mnew = LoadModel(sim.dropfilename, sim);
        sim.droploadrequest.store(false);

        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.dropfilename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          dynamic_obstacles.BindModel(m, d);
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      if (sim.uiloadrequest.load())
      {
        sim.uiloadrequest.fetch_sub(1);
        sim.LoadMessage(sim.filename);
        mjModel *mnew = LoadModel(sim.filename, sim);
        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          dynamic_obstacles.BindModel(m, d);
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      // sleep for 1 ms or yield, to let main thread run
      //  yield results in busy wait - which has better timing but kills battery life
      if (sim.run && sim.busywait)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          // running
          if (sim.run)
          {
            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std)
            {
              // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
              mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
              mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

              for (int i = 0; i < m->nu; i++)
              {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
              }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed)
            {
              // re-sync
              syncCPU = startCPU;
              syncSim = d->time;
              sim.speed_changed = false;

              // run single step, let next iteration deal with timing
              dynamic_obstacles.Step(m, d, m->opt.timestep);
              mj_step(m, d);
              stepped = true;
            }

            // in-sync: step until ahead of cpu
            else
            {
              bool measured = false;
              mjtNum prevSim = d->time;

              double refreshTime = simRefreshFraction / sim.refresh_rate;

              // step while sim lags behind cpu and within refreshTime
              while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                     mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
              {
                // measure slowdown before first step
                if (!measured && elapsedSim)
                {
                  sim.measured_slowdown =
                      std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                  measured = true;
                }

                // elastic band on base link
                if (param::config.enable_elastic_band == 1)
                {
                  if (elastic_band.enable_ && elastic_band.Released(d->time))
                  {
                    // Latch off once the ramp has run out, so the applied
                    // force is cleared exactly once and never re-applied.
                    elastic_band.enable_ = false;
                    d->xfrc_applied[param::config.band_attached_link] = 0.0;
                    d->xfrc_applied[param::config.band_attached_link + 1] = 0.0;
                    d->xfrc_applied[param::config.band_attached_link + 2] = 0.0;
                    std::cout << "elastic band released at t=" << d->time
                              << std::endl;
                  }
                  if (elastic_band.enable_)
                  {
                    std::vector<double> x = {d->qpos[0], d->qpos[1], d->qpos[2]};
                    std::vector<double> dx = {d->qvel[0], d->qvel[1], d->qvel[2]};

                    elastic_band.Advance(x, dx);

                    const double g = elastic_band.Gain(d->time);
                    d->xfrc_applied[param::config.band_attached_link] = g * elastic_band.f_[0];
                    d->xfrc_applied[param::config.band_attached_link + 1] = g * elastic_band.f_[1];
                    d->xfrc_applied[param::config.band_attached_link + 2] = g * elastic_band.f_[2];
                  }
                }

                // call mj_step
                dynamic_obstacles.Step(m, d, m->opt.timestep);
                mj_step(m, d);
                stepped = true;

                // break if reset
                if (d->time < prevSim)
                {
                  break;
                }
              }
            }

            // save current state to history buffer
            if (stepped)
            {
              sim.AddToHistory();
            }
          }

          // paused
          else
          {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }
        }
      } // release std::lock_guard<std::mutex>
    }
  }
  // Scripted command injection (Phase-4 §0): this machine has no gamepad, so
  // with use_joystick=0 the stock bridge never wires a joystick and the
  // 1 kHz axis_filter/Filter() seam is DEAD CODE. A ScriptedJoystick plays a
  // deterministic sim-time profile through the exact same joystick->update()
  // path. Test-only flag: enabled solely by env
  // UNITREE_MUJOCO_SCRIPTED_COMMANDS=<profile>; tracked config untouched;
  // inert when the variable is unset.
  //
  // Profile line: "t lx ly rx [buttons]", piecewise-constant hold. `buttons`
  // is an optional comma-separated list of key names held from that breakpoint
  // until the next; "-" or absent means none. Button support (interim block)
  // is what lets a scripted run drive g1_ctrl's FSM, whose transitions are
  // chords: L2+up -> FixStand, R2+A -> RLBase (deploy/robots/g1/main.cpp).
  // Names follow g1_pub.h's wireless_remote packing, where the FSM's "L2"/"R2"
  // are the joystick's LT/RT *axes*, not buttons — so those are driven to 1.0
  // (Axis::threshold is 0.5) and everything else is a Button<int>.
  //
  //   # t    lx   ly   rx   buttons
  //   0.0    0.0  0.0  0.0  -
  //   3.0    0.0  0.0  0.0  L2,up      <- enter FixStand
  //   3.5    0.0  0.0  0.0  -
  //   6.0    0.0  0.0  0.0  R2,A       <- enter RLBase
  //   6.5    0.0  0.0  0.0  -
  //   8.0    0.3  0.0  0.0  -          <- walk forward
  class ScriptedJoystick : public unitree::common::UnitreeJoystick
  {
  public:
    ScriptedJoystick(const std::string& profile_path,
                     JoystickAxisFilter axis_filter)
        : axis_filter_(std::move(axis_filter))
    {
      std::ifstream in(profile_path);
      if (!in)
      {
        std::cerr << "ScriptedJoystick: cannot open profile " << profile_path
                  << std::endl;
        exit(1);
      }
      std::string line;
      while (std::getline(in, line))
      {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        Point p{};
        if (!(ss >> p.t >> p.lx >> p.ly >> p.rx)) continue;
        std::string keys;
        if (ss >> keys && keys != "-") p.keys = ParseKeys(keys, profile_path);
        points_.push_back(p);
      }
      std::cout << "ScriptedJoystick: " << points_.size()
                << " breakpoints from " << profile_path << std::endl;
      if (axis_filter_)
      {
        // Same rule as the device joysticks: never low-pass blend a newly
        // safe command with an older command.
        lx.smooth = 1.0f;
        ly.smooth = 1.0f;
        rx.smooth = 1.0f;
      }
      // L2/R2 are Axis, and Axis's default smooth of 0.03 would take ~50
      // updates to cross the 0.5 press threshold — long enough that a short
      // scripted chord would never register. A scripted press is exact.
      LT.smooth = 1.0f;
      RT.smooth = 1.0f;
    }

    // Bit index per key name, in the order Apply() writes them.
    enum Key { kL2, kR2, kL1, kR1, kA, kB, kX, kY,
               kUp, kDown, kLeft, kRight, kStart, kSelect, kNumKeys };

    static uint32_t ParseKeys(const std::string& spec,
                              const std::string& profile_path)
    {
      static const std::pair<const char*, Key> kNames[] = {
          {"L2", kL2}, {"R2", kR2}, {"L1", kL1}, {"R1", kR1},
          {"LT", kL2}, {"RT", kR2}, {"LB", kL1}, {"RB", kR1},
          {"A", kA}, {"B", kB}, {"X", kX}, {"Y", kY},
          {"up", kUp}, {"down", kDown}, {"left", kLeft}, {"right", kRight},
          {"start", kStart}, {"select", kSelect}, {"back", kSelect}};
      uint32_t mask = 0;
      std::istringstream ks(spec);
      std::string name;
      while (std::getline(ks, name, ','))
      {
        if (name.empty()) continue;
        bool found = false;
        for (const auto& kv : kNames)
        {
          if (name == kv.first) { mask |= 1u << kv.second; found = true; break; }
        }
        if (!found)
        {
          // A typo'd key silently doing nothing is exactly how a scripted FSM
          // run turns into an unexplained "the robot never stood up".
          std::cerr << "ScriptedJoystick: unknown key '" << name << "' in "
                    << profile_path << std::endl;
          exit(1);
        }
      }
      return mask;
    }

    void update() override
    {
      // Sim-time sampled lock-free — same pre-existing benign-race class as
      // the bridge's own mjData reads (R-9, not worsened).
      const double t = d ? d->time : 0.0;
      std::array<float, 3> axes{0.0f, 0.0f, 0.0f};
      uint32_t keys = 0;
      for (const auto& p : points_)
      {
        if (p.t > t) break;
        axes = {p.lx, p.ly, p.rx};
        keys = p.keys;
      }
      if (axis_filter_) axes = axis_filter_(axes[0], axes[1], axes[2]);
      lx(axes[0]);
      ly(axes[1]);
      rx(axes[2]);
      ry(0.0f);
      // The chord keys are NOT routed through axis_filter_: DPCBF gates
      // locomotion commands, not mode changes (§10.5).
      const auto held = [keys](Key k) { return (keys >> k) & 1u; };
      LT(held(kL2) ? 1.0f : 0.0f);
      RT(held(kR2) ? 1.0f : 0.0f);
      LB(held(kL1)); RB(held(kR1));
      A(held(kA));   B(held(kB));   X(held(kX));    Y(held(kY));
      up(held(kUp)); down(held(kDown));
      left(held(kLeft)); right(held(kRight));
      start(held(kStart)); back(held(kSelect));
    }

  private:
    struct Point
    {
      double t;
      float lx, ly, rx;
      uint32_t keys = 0;
    };
    std::vector<Point> points_;
    JoystickAxisFilter axis_filter_;
  };

  // T1/T6 capture instrument (H-10): binary log of every Filter() call.
  std::unique_ptr<dpcbf_ros_adapter::FilterIoWriter> filter_io_log;
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d)
    {
      sim->Load(m, d, filename);
      dynamic_obstacles.BindModel(m, d);
      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);

  exit(0);
}

void *UnitreeSdk2BridgeThread(void *arg)
{
  // Wait for mujoco data
  while (true)
  {
    if (d)
    {
      std::cout << "Mujoco data is prepared" << std::endl;
      break;
    }
    usleep(500000);
  }

#ifdef UNITREE_MUJOCO_WITH_ROS2
  // ROS2 must initialize BEFORE the SDK2 ChannelFactory: rmw_cyclonedds
  // creates the CycloneDDS domain explicitly and hard-fails if it already
  // exists in-process (R-3, gate T10). The factory then JOINS the existing
  // domain (empty interface argument); the network interface is pinned for
  // both stacks via CYCLONEDDS_URI, derived from config.yaml by the bridge.
  SimRos2Bridge ros2_bridge;
  unitree::robot::ChannelFactory::Instance()->Init(param::config.domain_id, "");
#else
  unitree::robot::ChannelFactory::Instance()->Init(param::config.domain_id, param::config.interface);
#endif


  int body_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
  if (body_id < 0) {
    body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
  }
  param::config.band_attached_link = 6 * body_id;

  int dpcbf_body_id = mj_name2id(m, mjOBJ_BODY, safety_filter.base_body_name().c_str());
  if (dpcbf_body_id < 0) {
    std::cerr << "DPCBF base body was not found: "
              << safety_filter.base_body_name() << std::endl;
    return nullptr;
  }
  try {
    safety_filter.Initialize(m->opt.timestep);
    dpcbf_visualizer.Start();
  } catch (const std::exception& error) {
    std::cerr << "Failed to initialize DPCBF safety filter: " << error.what() << std::endl;
    return nullptr;
  }

  namespace dra = dpcbf_ros_adapter;
  const auto seam_limits = dra::SeamLimits::FromFilter(safety_filter);

  // The verbatim baseline obstacle acquisition (§10.4 kOracle): snapshot the
  // ground-truth manager and convert index-as-id. This exact loop is what
  // the pre-refactor lambda inlined; it is now the oracle provider.
  const auto oracle_provider = [] {
    std::vector<dpcbf::ObstacleState> obstacle_states;
    const auto obstacle_snapshot = dynamic_obstacles.Snapshot();
    obstacle_states.reserve(obstacle_snapshot.size());
    for (std::size_t obstacle_id = 0; obstacle_id < obstacle_snapshot.size();
         ++obstacle_id) {
      const auto& obstacle = obstacle_snapshot[obstacle_id];
      obstacle_states.push_back({obstacle.position[0], obstacle.position[1], obstacle.radius,
                                 obstacle.velocity[0], obstacle.velocity[1],
                                 static_cast<int>(obstacle_id)});
    }
    return obstacle_states;
  };

  // Mode selection (launch-selectable via env; DEFAULT REMAINS ORACLE, D5).
  auto mode = dra::ObstacleSource::Mode::kOracle;
  if (const char* mode_env = std::getenv("UNITREE_DPCBF_MODE")) {
    const std::string mode_str = mode_env;
    if (mode_str == "oracle") {
      mode = dra::ObstacleSource::Mode::kOracle;
    } else if (mode_str == "shadow") {
      mode = dra::ObstacleSource::Mode::kShadow;
    } else if (mode_str == "estimated") {
      mode = dra::ObstacleSource::Mode::kEstimated;
    } else {
      std::cerr << "UNITREE_DPCBF_MODE must be oracle|shadow|estimated, got "
                << mode_str << std::endl;
      return nullptr;
    }
  }
#ifndef UNITREE_MUJOCO_WITH_ROS2
  if (mode != dra::ObstacleSource::Mode::kOracle) {
    std::cerr << "shadow/estimated modes require the ROS2 build "
                 "(-DUNITREE_MUJOCO_WITH_ROS2=ON)" << std::endl;
    return nullptr;
  }
#endif
  if (const char* log_path = std::getenv("UNITREE_DPCBF_FILTER_LOG")) {
    filter_io_log = std::make_unique<dra::FilterIoWriter>(
        log_path, m->opt.timestep, static_cast<std::uint32_t>(mode));
    std::cout << "Filter I/O capture -> " << log_path << std::endl;
  }

#ifdef UNITREE_MUJOCO_WITH_ROS2
  // The seam behind ObstacleSource (§10.5). Constructed after SimRos2Bridge
  // so rclcpp is already initialized (T10 order). Appendix-A staleness
  // defaults; /obstacles_safe topic; /dpcbf/status diagnostics at 10 Hz.
  // The adapter node runs use_sim_time=false — it must never consume the
  // /clock this process publishes (§11.2 decision); every safety age is
  // sim-time t_query vs sim-time header stamps.
  static std::unique_ptr<dra::ObstacleSource> obstacle_source;
  {
    dra::ObstacleSource::Config source_config;
    source_config.mode = mode;
    source_config.oracle = oracle_provider;
    obstacle_source = std::make_unique<dra::ObstacleSource>(source_config);
  }
  std::cout << "DPCBF obstacle source mode: "
            << (mode == dra::ObstacleSource::Mode::kOracle ? "oracle"
                : mode == dra::ObstacleSource::Mode::kShadow ? "shadow"
                                                             : "estimated")
            << std::endl;

  JoystickAxisFilter axis_filter = [dpcbf_body_id, seam_limits](
                                       float lx, float ly, float rx) {
    const dpcbf::RobotState robot = ReadRobotGroundTruth(m, d, dpcbf_body_id);
    obstacle_source->SetRobotXY(robot.x, robot.y);
    const double t_query = d->time;  // sim time, same clock as header stamps
    auto snap = obstacle_source->GetObstacles(t_query);
    const auto desired = dra::AxesToDesired(lx, ly, rx, seam_limits);
    // §10.3 degrade ramp applied at the call site, before Filter() (§10.5).
    const auto scaled = dra::ScaleDesired(desired, snap.command_scale);
    const auto filtered = safety_filter.Filter(robot, scaled, snap.obstacles);
    dpcbf_visualizer.Update(robot, snap.obstacles, filtered);
    const auto axes = dra::CommandToAxes(filtered.command, seam_limits);
    if (filter_io_log && filter_io_log->enabled()) {
      dra::FilterIoPrefix prefix{};
      prefix.t = t_query;
      prefix.lx = lx; prefix.ly = ly; prefix.rx = rx;
      prefix.robot[0] = robot.x; prefix.robot[1] = robot.y;
      prefix.robot[2] = robot.phi;
      prefix.robot[3] = robot.sagittal_velocity;
      prefix.robot[4] = robot.lateral_velocity;
      prefix.desired[0] = desired.sagittal;
      prefix.desired[1] = desired.lateral;
      prefix.desired[2] = desired.yaw_rate;
      prefix.command_scale = snap.command_scale;
      prefix.age_s = std::isinf(snap.age_s) ? -1.0 : snap.age_s;
      prefix.staleness_state = static_cast<std::uint32_t>(snap.state);
      dra::FilterIoSuffix suffix{};
      suffix.out_command[0] = filtered.command.sagittal;
      suffix.out_command[1] = filtered.command.lateral;
      suffix.out_command[2] = filtered.command.yaw_rate;
      for (int i = 0; i < 3; ++i) suffix.acceleration[i] = filtered.acceleration[i];
      suffix.active_constraints = filtered.active_constraints;
      suffix.active_dpcbf_constraints = filtered.active_dpcbf_constraints;
      suffix.active_ecbf_constraints = filtered.active_ecbf_constraints;
      suffix.solved = filtered.solved ? 1 : 0;
      for (int i = 0; i < 3; ++i) suffix.out_axes[i] = axes[i];
      filter_io_log->Append(prefix, snap.obstacles, suffix);
    }
    return axes;
  };
#else
  // Non-ROS2 build: the baseline seam, byte-equivalent behavior (oracle
  // path inlined exactly as at f111cfa), plus the inert capture hook.
  JoystickAxisFilter axis_filter = [dpcbf_body_id, seam_limits, oracle_provider](
                                       float lx, float ly, float rx) {
    const auto desired = dra::AxesToDesired(lx, ly, rx, seam_limits);
    const auto obstacle_states = oracle_provider();
    const dpcbf::RobotState robot = ReadRobotGroundTruth(m, d, dpcbf_body_id);
    const auto filtered = safety_filter.Filter(robot, desired, obstacle_states);
    dpcbf_visualizer.Update(robot, obstacle_states, filtered);
    const auto axes = dra::CommandToAxes(filtered.command, seam_limits);
    if (filter_io_log && filter_io_log->enabled()) {
      dra::FilterIoPrefix prefix{};
      prefix.t = d->time;
      prefix.lx = lx; prefix.ly = ly; prefix.rx = rx;
      prefix.robot[0] = robot.x; prefix.robot[1] = robot.y;
      prefix.robot[2] = robot.phi;
      prefix.robot[3] = robot.sagittal_velocity;
      prefix.robot[4] = robot.lateral_velocity;
      prefix.desired[0] = desired.sagittal;
      prefix.desired[1] = desired.lateral;
      prefix.desired[2] = desired.yaw_rate;
      prefix.command_scale = 1.0;
      prefix.age_s = 0.0;
      prefix.staleness_state = 0;
      dra::FilterIoSuffix suffix{};
      suffix.out_command[0] = filtered.command.sagittal;
      suffix.out_command[1] = filtered.command.lateral;
      suffix.out_command[2] = filtered.command.yaw_rate;
      for (int i = 0; i < 3; ++i) suffix.acceleration[i] = filtered.acceleration[i];
      suffix.active_constraints = filtered.active_constraints;
      suffix.active_dpcbf_constraints = filtered.active_dpcbf_constraints;
      suffix.active_ecbf_constraints = filtered.active_ecbf_constraints;
      suffix.solved = filtered.solved ? 1 : 0;
      for (int i = 0; i < 3; ++i) suffix.out_axes[i] = axes[i];
      filter_io_log->Append(prefix, obstacle_states, suffix);
    }
    return axes;
  };
#endif

  std::unique_ptr<UnitreeSDK2BridgeBase> interface = nullptr;
  if (m->nu > NUM_MOTOR_IDL_GO) {
    interface = std::make_unique<G1Bridge>(m, d, axis_filter);
  } else {
    interface = std::make_unique<Go2Bridge>(m, d, axis_filter);
  }

  // Scripted band lowering (see ElasticBand::Gain above). Test-only, sim-time,
  // inert unless set — same discipline as the scripted commands below.
  if (const char* len = std::getenv("UNITREE_MUJOCO_BAND_LENGTH")) {
    elastic_band.length_ = std::atof(len);
  }
  if (const char* rel = std::getenv("UNITREE_MUJOCO_BAND_RELEASE")) {
    std::string spec(rel);
    const auto comma = spec.find(',');
    elastic_band.release_t_ = std::atof(spec.substr(0, comma).c_str());
    if (comma != std::string::npos) {
      elastic_band.release_ramp_ = std::atof(spec.substr(comma + 1).c_str());
    }
    std::cout << "elastic band: length " << elastic_band.length_
              << " m, release at t=" << elastic_band.release_t_
              << " s over " << elastic_band.release_ramp_ << " s" << std::endl;
  }

  // Scripted command injection (see ScriptedJoystick above): installed on
  // the PUBLIC lowstate/wireless_controller members before start(), so the
  // 1 kHz thread never observes a half-wired joystick. Without a device
  // (use_joystick=0) this is the only way the seam runs on this machine.
  std::shared_ptr<ScriptedJoystick> scripted_joystick;
  if (const char* profile = std::getenv("UNITREE_MUJOCO_SCRIPTED_COMMANDS")) {
    scripted_joystick = std::make_shared<ScriptedJoystick>(profile, axis_filter);
    if (auto* g1 = dynamic_cast<G1Bridge*>(interface.get())) {
      g1->lowstate->joystick = scripted_joystick;
      g1->wireless_controller->joystick = scripted_joystick;
    } else if (auto* go2 = dynamic_cast<Go2Bridge*>(interface.get())) {
      go2->lowstate->joystick = scripted_joystick;
      go2->wireless_controller->joystick = scripted_joystick;
    }
  }
  interface->start();

#ifdef UNITREE_MUJOCO_WITH_ROS2
  // ROS2 publishing reuses this otherwise-idle SDK2 bridge thread. mjData is
  // read without locks — the same pre-existing benign-race class as the
  // bridge's own reads (R-9: not worsened, no locks added).
  while (true)
  {
    ros2_bridge.SpinOnce(m, d, [] { return dynamic_obstacles.Snapshot(); });
    usleep(1000);
  }
#else
  while (true)
  {
    sleep(1);
  }
#endif
}
//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char *msg)
{
  rosetta_error_msg = msg;
}
#endif

// user keyboard callback
void user_key_cb(GLFWwindow* window, int key, int scancode, int act, int mods) {
  if (act==GLFW_PRESS)
  {
    if(param::config.enable_elastic_band == 1) {
      if (key==GLFW_KEY_9) {
        elastic_band.enable_ = !elastic_band.enable_;
      } else if (key==GLFW_KEY_7 || key==GLFW_KEY_UP) {
        elastic_band.length_ -= 0.1;
      } else if (key==GLFW_KEY_8 || key==GLFW_KEY_DOWN) {
        elastic_band.length_ += 0.1;
      }
    }
    if(key==GLFW_KEY_BACKSPACE) {
      mj_resetData(m, d);
      dynamic_obstacles.Reset(m, d);
      mj_forward(m, d);
    }
  }
}

// run event loop
int main(int argc, char **argv)
{

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg)
  {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // Load simulation configuration
  std::filesystem::path proj_dir = std::filesystem::path(getExecutableDir()).parent_path();
  param::config.load_from_yaml(proj_dir / "config.yaml");
  try {
    const auto dpcbf_config = proj_dir.parent_path() / "dpcbf/config/dpcbf_config.yaml";
    dynamic_obstacles.LoadConfig(dpcbf_config);
    safety_filter.LoadConfig(dpcbf_config);
    dpcbf_visualizer.LoadConfig(dpcbf_config);
  } catch (const std::exception& error) {
    std::cerr << "Failed to load dynamic obstacle configuration: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  param::helper(argc, argv);
  if(param::config.robot_scene.is_relative()) {
    param::config.robot_scene = proj_dir.parent_path() / param::config.robot_scene;
  }

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
    std::make_unique<mj::GlfwAdapter>(),
    &cam, &opt, &pert, /* is_passive = */ false);

  std::thread unitree_thread(UnitreeSdk2BridgeThread, nullptr);

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), param::config.robot_scene.c_str());
  // start simulation UI loop (blocking call)
  glfwSetKeyCallback(static_cast<mj::GlfwAdapter*>(sim->platform_ui.get())->window_,user_key_cb);
  sim->RenderLoop();
  physicsthreadhandle.join();

  pthread_exit(NULL);
  return 0;
}
