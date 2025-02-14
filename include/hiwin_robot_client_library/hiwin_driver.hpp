// Copyright 2024 HIWIN Technologies Corp.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef HIWIN_ROBOT_CLIENT_LIBRARY_HIWIN_DRIVER_HPP_
#define HIWIN_ROBOT_CLIENT_LIBRARY_HIWIN_DRIVER_HPP_

#include <string>
#include <vector>
#include <memory>

#include <hiwin_robot_client_library/socket/tcp_client.hpp>
#include <hiwin_robot_client_library/commander.hpp>
#include <hiwin_robot_client_library/event_cb.hpp>
#include <hiwin_robot_client_library/file_client.hpp>

namespace hrsdk
{
static const int COMMAND_PORT = 1503;
static const int EVENT_PORT = 1504;
static const int FILE_PORT = 1505;

class HIWINDriver
{
private:
  std::string robot_ip_;  // IP address of the robot
  std::string version_info_;
  std::string version_number_;

  std::unique_ptr<hrsdk::Commander> commander_;
  std::unique_ptr<hrsdk::EventCb> event_cb_;
  std::unique_ptr<hrsdk::FileClient> file_client_;

public:
  HIWINDriver(const std::string& robot_ip);
  ~HIWINDriver();

  bool connect();
  bool connect(int command_port, int event_port, int file_port);
  void disconnect();

  void getRobotVersion(std::string& version);
  bool isVersionGreaterOrEqual(const std::string& requiredVersion);

  void writeJointCommand(const std::vector<double>& positions);
  void writeTrajectorySplinePoint(const std::vector<double>& positions, const float goal_time);
  void writeTrajectorySplinePoint(const std::vector<double>& positions, const std::vector<double>& velocities,
                                  const float goal_time);
  void writeTrajectorySplinePoint(const std::vector<double>& positions, const std::vector<double>& velocities,
                                  const std::vector<double>& accelerations, const float goal_time);

  void motionAbort();
  void clearError();

  void getJointVelocity(std::vector<double>& velocities);
  void getJointEffort(std::vector<double>& efforts);
  void getJointPosition(std::vector<double>& positions);
  void getRobotMode(ControlMode& mode);
  void getErrorCode(int32_t& error_code);
  bool isEstopped();
  bool isDrivesPowered();
  bool isMotionPossible();
  bool isInMotion();
  bool isInError();
};

}  // namespace hrsdk

#endif  // HIWIN_ROBOT_CLIENT_LIBRARY_HIWIN_DRIVER_HPP_
