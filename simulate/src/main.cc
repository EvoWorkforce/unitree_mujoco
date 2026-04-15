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
#include "linear_trajectory_player.h"
#include "trajectory_scorer.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>

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
  ElasticBand() {};
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

  double stiffness_ = 200;
  double damping_ = 100;
  std::vector<double> point_ = {0, 0, 3};
  double length_ = 0.0;
  bool enable_ = true;
  std::vector<double> f_ = {0, 0, 0};
};
inline ElasticBand elastic_band;

// Global goal pose for DDS publishing (x, y, yaw) - relative to robot
std::vector<double> g_goal_pose = {0.0, 0.0, 0.0};

// Global key press for DDS publishing
std::string g_key_press = "";
bool g_key_press_updated = false;

// World-frame goal (set once when Space/Alt is released, continuously transformed to robot frame)
bool g_goal_world_valid = false;
double g_goal_world_x = 0.0;
double g_goal_world_y = 0.0;
double g_goal_world_yaw = 0.0;

// Goal mode: 0 = none, 1 = arrow (Space), 2 = site (Alt)
int g_goal_mode = 0;

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
  
  // control noise variables
  mjtNum *ctrlnoise = nullptr;
  
  // Robot goal pose (anonymous namespace)
  bool goal_site_active = false;
  bool goal_pose_active = false;
  int goal_site_id = -1;
  int goal_pose_base_id = -1;  // For arrow base geom
  int goal_pose_shaft_id = -1; // For arrow shaft geom
  std::vector<double> goal_pos = {0, 0, 0};
  std::vector<double> goal_pos_start = {0, 0, 0};
  
  const double goal_min_safe_distance = 0.0; // meters: stop this far short of the goal

  // Hidden position for markers (far off-screen)
  const double HIDDEN_POS[3] = {1000.0, 1000.0, -100.0};

  static mj::Simulate *g_sim_ptr = nullptr;

  // Helper function to hide site marker
  void hide_site_marker(mjModel *m)
  {
    if (goal_site_id >= 0 && m)
    {
      m->site_pos[3 * goal_site_id + 0] = HIDDEN_POS[0];
      m->site_pos[3 * goal_site_id + 1] = HIDDEN_POS[1];
      m->site_pos[3 * goal_site_id + 2] = HIDDEN_POS[2];
    }
  }

  // Helper function to hide arrow marker
  void hide_arrow_marker(mjModel *m)
  {
    if (goal_pose_base_id >= 0 && m)
    {
      m->geom_pos[3 * goal_pose_base_id + 0] = HIDDEN_POS[0];
      m->geom_pos[3 * goal_pose_base_id + 1] = HIDDEN_POS[1];
      m->geom_pos[3 * goal_pose_base_id + 2] = HIDDEN_POS[2];
    }
    if (goal_pose_shaft_id >= 0 && m)
    {
      m->geom_pos[3 * goal_pose_shaft_id + 0] = HIDDEN_POS[0];
      m->geom_pos[3 * goal_pose_shaft_id + 1] = HIDDEN_POS[1];
      m->geom_pos[3 * goal_pose_shaft_id + 2] = HIDDEN_POS[2];
    }
  }

  // Helper function to set goal object (site or arrow)
  void set_goal_object(bool is_arrow, bool active, mjModel *m, mjData *d)
  {
    if (!active)
    {
      return;
    }

    mjvCamera *cam = g_sim_ptr ? &g_sim_ptr->cam : nullptr;
    if (cam && cam->lookat && active)
    {
      double xpos, ypos;
      int win_width, win_height;

      glfwGetCursorPos(static_cast<mj::GlfwAdapter *>(g_sim_ptr->platform_ui.get())->window_, &xpos, &ypos);
      glfwGetWindowSize(static_cast<mj::GlfwAdapter *>(g_sim_ptr->platform_ui.get())->window_, &win_width, &win_height);

      mjtNum cam_pos[3], cam_forward[3], cam_up[3];
      mjv_cameraInModel(cam_pos, cam_forward, cam_up, &g_sim_ptr->scn);

      double ndc_x = (2.0 * xpos) / win_width - 1.0;
      double ndc_y = 1.0 - (2.0 * ypos) / win_height;
      mjtNum cam_right[3];

      mju_cross(cam_right, cam_forward, cam_up);
      mju_normalize3(cam_right);
      mju_normalize3(cam_up);

      double fov_y = 45.0 * M_PI / 180.0;
      double aspect = (double)win_width / (double)win_height;
      double tan_fov_y = tan(fov_y / 2.0);
      double ray_dir_cam[3] = {ndc_x * aspect * tan_fov_y, ndc_y * tan_fov_y, -1.0};
      double norm = sqrt(ray_dir_cam[0] * ray_dir_cam[0] + ray_dir_cam[1] * ray_dir_cam[1] + ray_dir_cam[2] * ray_dir_cam[2]);

      ray_dir_cam[0] /= norm;
      ray_dir_cam[1] /= norm;
      ray_dir_cam[2] /= norm;
      mjtNum ray_dir_model[3];

      for (int i = 0; i < 3; ++i)
      {
        ray_dir_model[i] = ray_dir_cam[0] * cam_right[i] + ray_dir_cam[1] * cam_up[i] + ray_dir_cam[2] * (-cam_forward[i]);
      }

      mju_normalize3(ray_dir_model);

      if (fabs(ray_dir_model[2]) > 1e-8)
      {
        double t = -cam_pos[2] / ray_dir_model[2];
        if (t >= 0)
        {
          double x = cam_pos[0] + t * ray_dir_model[0];
          double y = cam_pos[1] + t * ray_dir_model[1];
          double z = 0.0;

          if (!is_arrow)
          {
            goal_pos = {x, y, z};
            m->site_pos[3 * goal_site_id + 0] = goal_pos[0];
            m->site_pos[3 * goal_site_id + 1] = goal_pos[1];
            m->site_pos[3 * goal_site_id + 2] = goal_pos[2];
          }
          else
          {
            // Arrow logic: on initial press, set start_pos; while held, update direction
            if (goal_pos_start.empty())
            {
              goal_pos_start = {x, y, z};
            }
            goal_pos = {x, y, z};

            // Compute direction vector and length
            double dx = goal_pos[0] - goal_pos_start[0];
            double dy = goal_pos[1] - goal_pos_start[1];
            double dz = goal_pos[2] - goal_pos_start[2];
            double norm_dir = sqrt(dx * dx + dy * dy + dz * dz);

            if (norm_dir > 1e-6)
            {
              // Arrow points along (dx, dy, dz) from pos_start
              // Assume default arrow points along +x, compute rotation to (dx, dy, dz)
              double axis[3] = {0, 0, 1}; // z-up
              double angle = atan2(dy, dx);
              double quat[4];
              mju_axisAngle2Quat(quat, axis, angle);

              // Apply base rotation to shaft quaternion
              double base_quat[4] = {0, 0.7071068, 0, 0.7071068};
              double shaft_quat[4];
              mju_mulQuat(shaft_quat, quat, base_quat);

              // Calculate normalized position for arrow shaft
              double shaft_half_length = 0.2;
              double shaft_pos[3] = {goal_pos_start[0] + dx * shaft_half_length / norm_dir,
                                     goal_pos_start[1] + dy * shaft_half_length / norm_dir,
                                     goal_pos_start[2] + dz * shaft_half_length / norm_dir};

              // Set orientation for both base and shaft geoms
              if (goal_pose_base_id >= 0 && goal_pose_shaft_id >= 0)
              {
                // Base position and orientation
                m->geom_pos[3 * goal_pose_base_id + 0] = goal_pos_start[0];
                m->geom_pos[3 * goal_pose_base_id + 1] = goal_pos_start[1];
                m->geom_pos[3 * goal_pose_base_id + 2] = goal_pos_start[2];
                m->geom_quat[4 * goal_pose_base_id + 0] = quat[0];
                m->geom_quat[4 * goal_pose_base_id + 1] = quat[1];
                m->geom_quat[4 * goal_pose_base_id + 2] = quat[2];
                m->geom_quat[4 * goal_pose_base_id + 3] = quat[3];
                // Shaft position and orientation (with base rotation applied)
                m->geom_pos[3 * goal_pose_shaft_id + 0] = shaft_pos[0];
                m->geom_pos[3 * goal_pose_shaft_id + 1] = shaft_pos[1];
                m->geom_pos[3 * goal_pose_shaft_id + 2] = shaft_pos[2];
                m->geom_quat[4 * goal_pose_shaft_id + 0] = shaft_quat[0];
                m->geom_quat[4 * goal_pose_shaft_id + 1] = shaft_quat[1];
                m->geom_quat[4 * goal_pose_shaft_id + 2] = shaft_quat[2];
                m->geom_quat[4 * goal_pose_shaft_id + 3] = shaft_quat[3];
              }
            }
          }
        }
      }
      mj_forward(m, d);
    }
  }

  // Removed duplicate g_sim_ptr declaration

  using Seconds = std::chrono::duration<double>;

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
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
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
          // Update goal_site (Right Alt) and goal_pose arrow (Space)
          set_goal_object(false, goal_site_active, m, d);
          set_goal_object(true, goal_pose_active, m, d);

          // Continuously update world goal from site mode (Right Alt held)
          if (goal_site_active && goal_pos.size() >= 2 && d)
          {
            double robot_x = d->qpos[0];
            double robot_y = d->qpos[1];
            double dx_to_goal = goal_pos[0] - robot_x;
            double dy_to_goal = goal_pos[1] - robot_y;

            g_goal_world_x = goal_pos[0];
            g_goal_world_y = goal_pos[1];
            g_goal_world_yaw = atan2(dy_to_goal, dx_to_goal);
            g_goal_world_valid = true;
          }

          // Advance trajectory player if active
          if (trajectory_player.IsActive() && d)
          {
            trajectory_player.Advance(m->opt.timestep);
            
            // Get trajectory goal in world frame
            double traj_x, traj_y, traj_yaw;
            trajectory_player.GetCurrentGoal(traj_x, traj_y, traj_yaw);
            
            // Set as world goal (will be transformed to robot frame below)
            g_goal_world_x = traj_x;
            g_goal_world_y = traj_y;
            g_goal_world_yaw = traj_yaw;
            g_goal_world_valid = true;
            
            // Update arrow to visualize the current trajectory goal with yaw
            if (goal_pose_base_id >= 0 && goal_pose_shaft_id >= 0 && m)
            {
              // Arrow orientation from yaw angle
              double axis[3] = {0, 0, 1}; // z-up
              double quat[4];
              mju_axisAngle2Quat(quat, axis, traj_yaw);

              // Apply base rotation to shaft quaternion
              double base_quat[4] = {0, 0.7071068, 0, 0.7071068};
              double shaft_quat[4];
              mju_mulQuat(shaft_quat, quat, base_quat);

              // Arrow shaft position (offset along arrow direction)
              double shaft_length = 0.2;
              double shaft_pos[3] = {
                traj_x + shaft_length * cos(traj_yaw),
                traj_y + shaft_length * sin(traj_yaw),
                0.05  // Slightly above ground
              };

              // Base (cone) position and orientation
              m->geom_pos[3 * goal_pose_base_id + 0] = traj_x;
              m->geom_pos[3 * goal_pose_base_id + 1] = traj_y;
              m->geom_pos[3 * goal_pose_base_id + 2] = 0.05;  // Slightly above ground
              m->geom_quat[4 * goal_pose_base_id + 0] = quat[0];
              m->geom_quat[4 * goal_pose_base_id + 1] = quat[1];
              m->geom_quat[4 * goal_pose_base_id + 2] = quat[2];
              m->geom_quat[4 * goal_pose_base_id + 3] = quat[3];

              // Shaft (cylinder) position and orientation
              m->geom_pos[3 * goal_pose_shaft_id + 0] = shaft_pos[0];
              m->geom_pos[3 * goal_pose_shaft_id + 1] = shaft_pos[1];
              m->geom_pos[3 * goal_pose_shaft_id + 2] = shaft_pos[2];
              m->geom_quat[4 * goal_pose_shaft_id + 0] = shaft_quat[0];
              m->geom_quat[4 * goal_pose_shaft_id + 1] = shaft_quat[1];
              m->geom_quat[4 * goal_pose_shaft_id + 2] = shaft_quat[2];
              m->geom_quat[4 * goal_pose_shaft_id + 3] = shaft_quat[3];
            }
          }
          else if (!goal_pose_active)
          {
            // Hide arrow when trajectory is not active and manual goal not active
            hide_arrow_marker(m);
          }

          // Continuously update relative goal pose from world-frame goal
          if (g_goal_world_valid && d)
          {
            // Get robot's current pose from qpos (floating base: x, y, z, qw, qx, qy, qz)
            double robot_x = d->qpos[0];
            double robot_y = d->qpos[1];
            double qw = d->qpos[3];
            double qx = d->qpos[4];
            double qy = d->qpos[5];
            double qz = d->qpos[6];
            double robot_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

            // Transform goal position to robot-relative frame
            double dx_to_goal = g_goal_world_x - robot_x;
            double dy_to_goal = g_goal_world_y - robot_y;
            double cos_yaw = cos(-robot_yaw);
            double sin_yaw = sin(-robot_yaw);
            double rel_x = cos_yaw * dx_to_goal - sin_yaw * dy_to_goal;
            double rel_y = sin_yaw * dx_to_goal + cos_yaw * dy_to_goal;

            // Enforce minimum safe distance from goal:
            // - If the goal is within 1m, publish (0, 0, yaw)
            // - Otherwise, publish a point that is 1m closer to the robot along the goal direction
            const double rel_dist = sqrt(rel_x * rel_x + rel_y * rel_y);
            if (rel_dist <= goal_min_safe_distance)
            {
              rel_x = 0.0;
              rel_y = 0.0;
            }
            else
            {
              const double scale = (rel_dist - goal_min_safe_distance) / rel_dist;
              rel_x *= scale;
              rel_y *= scale;
            }

            // Relative yaw (goal yaw - robot yaw), normalized to [-pi, pi]
            double rel_yaw = g_goal_world_yaw - robot_yaw;
            while (rel_yaw > M_PI)
              rel_yaw -= 2.0 * M_PI;
            while (rel_yaw < -M_PI)
              rel_yaw += 2.0 * M_PI;

            g_goal_pose = {rel_x, rel_y, rel_yaw};

            // Update trajectory scorer with error measurements (if trajectory is active)
            if (trajectory_player.IsActive())
            {
              trajectory_scorer.UpdatePositionError(rel_x, rel_y);
              trajectory_scorer.UpdateOrientationError(rel_yaw);
            }

            // Check if trajectory just finished (must be outside IsActive check!)
            static bool was_active = false;
            if (was_active && !trajectory_player.IsActive() && trajectory_player.IsFinished())
            {
              // Trajectory completed - print scoring summary
              trajectory_scorer.PrintSummary();
            }
            was_active = trajectory_player.IsActive();
          }

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
                  if (elastic_band.enable_)
                  {
                    std::vector<double> x = {d->qpos[0], d->qpos[1], d->qpos[2]};
                    std::vector<double> dx = {d->qvel[0], d->qvel[1], d->qvel[2]};

                    elastic_band.Advance(x, dx);

                    d->xfrc_applied[param::config.band_attached_link] = elastic_band.f_[0];
                    d->xfrc_applied[param::config.band_attached_link + 1] = elastic_band.f_[1];
                    d->xfrc_applied[param::config.band_attached_link + 2] = elastic_band.f_[2];
                  }
                }

                // call mj_step
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
      mj_forward(m, d);

      // Set goal_site_id to the correct site index
      goal_site_id = mj_name2id(m, mjOBJ_SITE, "goal_site");
      if (goal_site_id < 0)
      {
        std::cerr << "ERROR: goal_site not found in model!" << std::endl;
      }
      // Set goal_pose_base_id and goal_pose_shaft_id to the correct geom indices (for arrow)
      goal_pose_base_id = mj_name2id(m, mjOBJ_GEOM, "goal_pose_arrow_base");
      if (goal_pose_base_id < 0)
      {
        std::cerr << "ERROR: goal_pose_arrow_base geom not found in model!" << std::endl;
      }
      goal_pose_shaft_id = mj_name2id(m, mjOBJ_GEOM, "goal_pose_arrow_shaft");
      if (goal_pose_shaft_id < 0)
      {
        std::cerr << "ERROR: goal_pose_arrow_shaft geom not found in model!" << std::endl;
      }
      // Store these IDs in global variables if needed for later use

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

// Load trajectory configuration from YAML file
bool LoadTrajectoryConfig(const std::string &yaml_path)
{
  try
  {
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    // Load parameters
    if (config["parameters"])
    {
      auto params = config["parameters"];
      if (params["max_velocity"])
        trajectory_player.max_velocity_ = params["max_velocity"].as<double>();
      if (params["max_acceleration"])
        trajectory_player.max_acceleration_ = params["max_acceleration"].as<double>();
      if (params["max_angular_velocity"])
        trajectory_player.max_angular_velocity_ = params["max_angular_velocity"].as<double>();
      if (params["max_angular_acceleration"])
        trajectory_player.max_angular_acceleration_ = params["max_angular_acceleration"].as<double>();
    }
    
    // Load segments
    trajectory_player.ClearSegments();
    if (config["segments"] && config["segments"].IsSequence())
    {
      for (const auto &segment : config["segments"])
      {
        if (segment.IsSequence() && segment.size() >= 4)
        {
          double x_start = segment[0].as<double>();
          double y_start = segment[1].as<double>();
          double x_end = segment[2].as<double>();
          double y_end = segment[3].as<double>();
          double yaw_start = segment.size() > 4 ? segment[4].as<double>() : 0.0;
          double yaw_end = segment.size() > 5 ? segment[5].as<double>() : 0.0;
          
          trajectory_player.AddSegment(x_start, y_start, x_end, y_end, yaw_start, yaw_end);
        }
      }
    }
    
    // Print loaded configuration
    std::cout << "Trajectory configuration loaded from: " << yaml_path << std::endl;
    std::cout << "  Segments: " << trajectory_player.GetSegmentCount() << std::endl;
    std::cout << "  Linear: max_vel=" << trajectory_player.max_velocity_ 
              << " m/s, max_accel=" << trajectory_player.max_acceleration_ << " m/s^2" << std::endl;
    std::cout << "  Angular: max_vel=" << trajectory_player.max_angular_velocity_ 
              << " rad/s, max_accel=" << trajectory_player.max_angular_acceleration_ << " rad/s^2" << std::endl;
    std::cout << "  Trajectory visualized with directional arrow (shows position & yaw)" << std::endl;
    std::cout << "Press 'Q' to play/stop trajectory sequence" << std::endl;
    
    return true;
  }
  catch (const YAML::Exception &e)
  {
    std::cerr << "Error loading trajectory config: " << e.what() << std::endl;
    std::cerr << "Using default hardcoded trajectory" << std::endl;
    return false;
  }
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

  unitree::robot::ChannelFactory::Instance()->Init(param::config.domain_id, param::config.interface);

  // Try to load trajectory from YAML file
  std::filesystem::path proj_dir = std::filesystem::path(getExecutableDir()).parent_path();
  std::filesystem::path trajectory_yaml = proj_dir / "trajectory.yaml";
  
  bool loaded = false;
  if (std::filesystem::exists(trajectory_yaml))
  {
    loaded = LoadTrajectoryConfig(trajectory_yaml.string());
  }
  else
  {
    std::cerr << "Trajectory config file not found: " << trajectory_yaml << std::endl;
  }
  
  // Fallback to hardcoded trajectory if YAML loading failed
  if (!loaded)
  {
    std::cout << "Initializing default trajectory (square path)" << std::endl;
    trajectory_player.ClearSegments();
    trajectory_player.AddSegment(0.0, 0.0, 2.0, 0.0, 0.0, 0.0);           // Segment 1: East (facing 0°)
    trajectory_player.AddSegment(2.0, 0.0, 2.0, 2.0, 0.0, M_PI/2);        // Segment 2: North (turn to 90°)
    trajectory_player.AddSegment(2.0, 2.0, 0.0, 2.0, M_PI/2, M_PI);       // Segment 3: West (turn to 180°)
    trajectory_player.AddSegment(0.0, 2.0, 0.0, 0.0, M_PI, -M_PI/2);      // Segment 4: South (turn to -90°)
    trajectory_player.max_velocity_ = 0.2;
    trajectory_player.max_acceleration_ = 0.4;
    trajectory_player.max_angular_velocity_ = 0.5;
    trajectory_player.max_angular_acceleration_ = 0.3;
    std::cout << "  Trajectory sequence initialized with " << trajectory_player.GetSegmentCount() 
              << " segments (square path with yaw control)" << std::endl;
    std::cout << "  Linear: max_vel=" << trajectory_player.max_velocity_ 
              << " m/s, max_accel=" << trajectory_player.max_acceleration_ << " m/s^2" << std::endl;
    std::cout << "  Angular: max_vel=" << trajectory_player.max_angular_velocity_ 
              << " rad/s, max_accel=" << trajectory_player.max_angular_acceleration_ << " rad/s^2" << std::endl;
    std::cout << "  Trajectory visualized with directional arrow (shows position & yaw)" << std::endl;
    std::cout << "Press 'Q' to play/stop trajectory sequence" << std::endl;
  }

  int body_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
  if (body_id < 0)
  {
    body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
  }
  param::config.band_attached_link = 6 * body_id;

  std::unique_ptr<UnitreeSDK2BridgeBase> interface = nullptr;
  if (m->nu > NUM_MOTOR_IDL_GO)
  {
    interface = std::make_unique<G1Bridge>(m, d);
  }
  else
  {
    interface = std::make_unique<Go2Bridge>(m, d);
  }
  interface->start();

  while (true)
  {
    sleep(1);
  }
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

void reset_goal()
{
  // Stop trajectory playback if active
  if (trajectory_player.IsActive())
  {
    trajectory_player.Stop();
    std::cout << "Trajectory playback stopped (reset)" << std::endl;
  }
  
  // Reset goal pose to zero (not empty!)
  g_goal_pose = {0.0, 0.0, 0.0};
  goal_pos.clear();
  goal_pos_start.clear();
  goal_site_active = false;
  goal_pose_active = false;
  
  // Reset world-frame goal
  g_goal_world_x = 0.0;
  g_goal_world_y = 0.0;
  g_goal_world_yaw = 0.0;
  g_goal_world_valid = false;
  
  // Hide both markers
  hide_site_marker(m);
  hide_arrow_marker(m);
  
  std::cout << "Reset: goal_pose set to (0, 0, 0)" << std::endl;
}

void reset_robot()
{
    mj_resetData(m, d);
    mj_forward(m, d);
    reset_goal();
}

// user keyboard callback
void user_key_cb(GLFWwindow *window, int key, int scancode, int act, int mods)
{
  // Publish key press event
  if (act == GLFW_PRESS && key != GLFW_KEY_E)
  {
    const char* key_name = glfwGetKeyName(key, scancode);
    if (key_name)
    {
      g_key_press = std::string(key_name);
    }
    else
    {
      // For special keys, use the key code
      g_key_press = "KEY_" + std::to_string(key);
    }
    g_key_press_updated = true;
    std::cout << "Key pressed: " << g_key_press << std::endl;
  }

  // E key: Play/stop trajectory
  if (key == GLFW_KEY_E && act == GLFW_PRESS)
  {
    reset_robot();

    if (trajectory_player.IsActive())
    {
      trajectory_player.Stop();
      // Print scoring summary when manually stopped
      if (trajectory_scorer.HasData())
      {
        trajectory_scorer.PrintSummary();
      }
      std::cout << "Trajectory playback stopped" << std::endl;
    }
    else
    {
      trajectory_player.Start();
      // Reset scorer when starting trajectory
      trajectory_scorer.Reset();
      std::cout << "Trajectory playback started" << std::endl;
    }
  }

  if (param::config.enable_elastic_band == 1)
  {
    if (key == GLFW_KEY_9 && act == GLFW_PRESS)
    {
      elastic_band.enable_ = !elastic_band.enable_;
    }
    else if ((key == GLFW_KEY_7 || key == GLFW_KEY_UP) && act == GLFW_PRESS)
    {
      elastic_band.length_ -= 0.1;
    }
    else if ((key == GLFW_KEY_8 || key == GLFW_KEY_DOWN) && act == GLFW_PRESS)
    {
      elastic_band.length_ += 0.1;
    }
  }
  if (key == GLFW_KEY_BACKSPACE && act == GLFW_PRESS)
  {
    reset_robot();
  }
  // Set goal_site with Right Alt, goal_pose arrow with Space
  if (key == GLFW_KEY_RIGHT_ALT)
  {
    if (act == GLFW_PRESS)
    {
      std::cout << "Right Alt PRESS" << std::endl;
      goal_site_active = true;
      goal_pose_active = false;
      // Hide arrow marker when site mode is active
      hide_arrow_marker(m);
      g_goal_mode = 2; // Site mode
    }
    else if (act == GLFW_RELEASE)
    {
      std::cout << "Right Alt RELEASE" << std::endl;
      goal_site_active = false;
    }
  }
  if (key == GLFW_KEY_SPACE)
  {
    if (act == GLFW_PRESS)
    {
      std::cout << "Space PRESS" << std::endl;
      goal_pose_active = true;
      goal_site_active = false;
      goal_pos_start.clear(); // Reset start position for new arrow
      // Hide site marker when arrow mode is active
      hide_site_marker(m);
    }
    else if (act == GLFW_RELEASE)
    {
      std::cout << "Space RELEASE" << std::endl;
      goal_pose_active = false;
      // Store world-frame goal (will be continuously transformed to robot frame in physics loop)
      if (!goal_pos_start.empty() && goal_pos.size() >= 2 && goal_pos_start.size() >= 2)
      {
        // Compute world-frame goal yaw from arrow direction
        double dx_world = goal_pos[0] - goal_pos_start[0];
        double dy_world = goal_pos[1] - goal_pos_start[1];

        g_goal_world_x = goal_pos_start[0];
        g_goal_world_y = goal_pos_start[1];
        g_goal_world_yaw = atan2(dy_world, dx_world);
        g_goal_world_valid = true;
        g_goal_mode = 1; // Arrow mode

        std::cout << "Goal: x=" << g_goal_world_x << ", y=" << g_goal_world_y << ", yaw=" << g_goal_world_yaw << std::endl;
      }
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
  param::helper(argc, argv);
  if (param::config.robot_scene.is_relative())
  {
    param::config.robot_scene = proj_dir.parent_path() / "unitree_robots" / param::config.robot / param::config.robot_scene;
  }

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false);

  sim->ui0_enable = 0;  // Disable left panel
  sim->ui1_enable = 0;  // Disable right panel

  // Set MuJoCo UI user pointer (required for internal event handling)
  glfwSetWindowUserPointer(static_cast<mj::GlfwAdapter *>(sim->platform_ui.get())->window_, sim->platform_ui.get());
  // Set static pointer for custom mouse callback
  g_sim_ptr = sim.get();

  std::thread unitree_thread(UnitreeSdk2BridgeThread, nullptr);

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), param::config.robot_scene.c_str());
  // start simulation UI loop (blocking call)
  glfwSetKeyCallback(static_cast<mj::GlfwAdapter *>(sim->platform_ui.get())->window_, user_key_cb);
  sim->RenderLoop();
  physicsthreadhandle.join();

  pthread_exit(NULL);
  return 0;
}
