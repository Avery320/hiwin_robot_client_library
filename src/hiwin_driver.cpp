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

#include <regex>
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <bits/stdc++.h>

#include <hiwin_robot_client_library/hiwin_driver.hpp>

namespace hrsdk
{
HIWINDriver::HIWINDriver(const std::string& robot_ip) : robot_ip_(robot_ip)
{
}

HIWINDriver::~HIWINDriver()
{
  disconnect();  // Ensure disconnection on destruction
}

bool HIWINDriver::connect()
{
  return connect(hrsdk::COMMAND_PORT, hrsdk::EVENT_PORT, hrsdk::FILE_PORT);
}

bool HIWINDriver::connect(int command_port, int event_port, int file_port)
{
  commander_.reset(new hrsdk::Commander(robot_ip_, command_port));
  if (!commander_->connect())
  {
    return false;
  }

  event_cb_.reset(new hrsdk::EventCb(robot_ip_, event_port));
  if (!event_cb_->connect())
  {
    return false;
  }

  file_client_.reset(new hrsdk::FileClient(robot_ip_, file_port));
  if (!file_client_->connect())
  {
    return false;
  }

  commander_->GetRobotVersion(version_info_);
  std::cout << version_info_ << std::endl;

  commander_->getPermissions();
  commander_->setLogLevel(LogLevels::SetCommand);

  commander_->setRobotMode(ControlMode::Auto);
  commander_->setPtpSpeed(100);
  commander_->setOverrideRatio(100);

  commander_->setServoAmpState(true);

  return true;
}

void HIWINDriver::disconnect()
{
}

void HIWINDriver::getRobotVersion(std::string& version)
{
  std::regex version_regex(R"(HRDLL (\d+\.\d+\.\d+))");
  std::smatch match;

  if (std::regex_search(version_info_, match, version_regex))
  {
    version_number_ = match[1].str();
  }
  else
  {
    version_regex = std::regex(R"(HRSS (\d+\.\d+\.\d+))");
    if (std::regex_search(version_info_, match, version_regex))
    {
      version_number_ = match[1].str();
    }
    else
    {
      version_number_ = "0.0.0";
    }
  }

  version = version_number_;
}

bool HIWINDriver::isVersionGreaterOrEqual(const std::string& requiredVersion)
{
  std::vector<int> versionParts, requiredParts;

  std::stringstream versionStream(version_number_);
  std::stringstream requiredStream(requiredVersion);
  std::string part;

  while (std::getline(versionStream, part, '.'))
  {
    versionParts.push_back(std::stoi(part));
  }
  while (std::getline(requiredStream, part, '.'))
  {
    requiredParts.push_back(std::stoi(part));
  }

  for (size_t i = 0; i < std::max(versionParts.size(), requiredParts.size()); ++i)
  {
    int versionPart = (i < versionParts.size()) ? versionParts[i] : 0;
    int requiredPart = (i < requiredParts.size()) ? requiredParts[i] : 0;

    if (versionPart > requiredPart)
    {
      return true;
    }
    if (versionPart < requiredPart)
    {
      return false;
    }
  }
  return true;
}

void HIWINDriver::getRobotMode(ControlMode& mode)
{
  commander_->getRobotMode(mode);
}

bool HIWINDriver::isEstopped()
{
  return false;
}

bool HIWINDriver::isDrivesPowered()
{
  bool state = false;
  commander_->getServoAmpState(state);
  return state;
}

bool HIWINDriver::isMotionPossible()
{
  if (!isDrivesPowered() || !isInError() || !commander_->isRemoteMode())
  {
    return false;
  }
  return true;
}

bool HIWINDriver::isInMotion()
{
  MotionStatus robotStatus;

  commander_->getMotionState(robotStatus);
  if (robotStatus == MotionStatus::Moving)
  {
    return true;
  }
  return false;
}

bool HIWINDriver::isInError()
{
  std::vector<std::string> error_list;

  commander_->getErrorCode(error_list);
  if (error_list.empty())
  {
    return false;
  }
  return true;
}

void HIWINDriver::getErrorCode(int32_t& error_code)
{
  std::vector<std::string> error_list;

  commander_->getErrorCode(error_list);
  if (error_list.empty())
  {
    error_code = 0;
    return;
  }

  std::string last_error = error_list.back();

  std::regex pattern(R"(Err([0-9A-Fa-f]{2})-([0-9A-Fa-f]{2})-([0-9A-Fa-f]{2}))");
  std::smatch matches;

  if (std::regex_match(last_error, matches, pattern))
  {
    uint16_t first = std::stoi(matches[1].str(), nullptr, 16);
    uint16_t second = std::stoi(matches[2].str(), nullptr, 16);
    uint16_t third = std::stoi(matches[3].str(), nullptr, 16);

    error_code = (first << 16) | (second << 8) | (third << 0);
  }
  return;
}

void HIWINDriver::getJointVelocity(std::vector<double>& velocities)
{
  if (velocities.size() > 9)
  {
    return;
  }

  double value[6];
  double extra_value[3];

  if (commander_->getActualRPM(value) != 0)
  {
    return;
  }
  for (size_t i = 0; i < 6; i++)
  {
    velocities.at(i) = value[i];
  }

  if (velocities.size() > 6)
  {
    if (commander_->getExtActualRPM(extra_value) != 0)
    {
      return;
    }
    for (size_t i = 0; i < velocities.size() - 6; i++)
    {
      velocities.at(6 + i) = extra_value[i];
    }
  }

  return;
}

void HIWINDriver::getJointEffort(std::vector<double>& efforts)
{
  if (efforts.size() > 9)
  {
    return;
  }

  double value[6];
  double extra_value[6];
  if (commander_->getActualCurrent(value) != 0)
  {
    return;
  }
  for (size_t i = 0; i < 6; i++)
  {
    efforts.at(i) = value[i];
  }

  if (efforts.size() > 6)
  {
  }

  return;
}

void HIWINDriver::getJointPosition(std::vector<double>& positions)
{
  if (positions.size() > 9)
  {
    return;
  }

  double value[6];
  double extra_value[3];
  if (commander_->getActualPosition(value) != 0)
  {
    return;
  }
  for (size_t i = 0; i < 6; i++)
  {
    positions.at(i) = value[i];
  }

  if (positions.size() > 6)
  {
    if (commander_->getExtActualPosition(extra_value) != 0)
    {
      return;
    }
    for (size_t i = 0; i < positions.size() - 6; i++)
    {
      positions.at(6 + i) = extra_value[i];
    }
  }

  return;
}

void HIWINDriver::writeJointCommand(const std::vector<double>& positions)
{
  if (positions.size() > 9)
  {
    return;
  }

  double value[9] = { 0.0 };
  std::copy(positions.begin(), positions.end(), value);

  if (positions.size() > 6)
  {
    commander_->extPtpJoint(value);
  }
  else
  {
    commander_->ptpJoint(value);
  }
}

void HIWINDriver::writeTrajectorySplinePoint(const std::vector<double>& positions, const float goal_time)
{
  if (positions.size() > 9)
  {
    return;
  }

  double p[9] = { 0.0 };
  std::copy(positions.begin(), positions.end(), p);

  commander_->linearSplinePoint(p, goal_time);
}

void HIWINDriver::writeTrajectorySplinePoint(const std::vector<double>& positions,
                                             const std::vector<double>& velocities, const float goal_time)
{
  if (positions.size() > 9 || velocities.size() > 9)
  {
    return;
  }

  double p[9] = { 0.0 };
  std::copy(positions.begin(), positions.end(), p);

  double v[9] = { 0.0 };
  std::copy(velocities.begin(), velocities.end(), v);

  commander_->CubicSplinePoint(p, v, goal_time);
}

void HIWINDriver::writeTrajectorySplinePoint(const std::vector<double>& positions,
                                             const std::vector<double>& velocities,
                                             const std::vector<double>& accelerations, const float goal_time)
{
  if (positions.size() > 9 || velocities.size() > 9 || accelerations.size() > 9)
  {
    return;
  }

  double p[9] = { 0.0 };
  std::copy(positions.begin(), positions.end(), p);

  double v[9] = { 0.0 };
  std::copy(velocities.begin(), velocities.end(), v);

  double a[9] = { 0.0 };
  std::copy(accelerations.begin(), accelerations.end(), a);

  commander_->QuintSplinePoint(p, v, a, goal_time);
}

void HIWINDriver::motionAbort()
{
  commander_->motionAbort();
}

void HIWINDriver::clearError()
{
  commander_->clearError();
  commander_->setServoAmpState(true);
}

}  // namespace hrsdk
