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
  commander_.reset(new hrsdk::Commander(robot_ip_, hrsdk::COMMAND_PORT));
  if (!commander_->connect())
  {
    return false;
  }

  event_cb_.reset(new hrsdk::EventCb(robot_ip_, hrsdk::EVENT_PORT));
  if (!event_cb_->connect())
  {
    return false;
  }

  file_client_.reset(new hrsdk::FileClient(robot_ip_, hrsdk::FILE_PORT));
  if (!file_client_->connect())
  {
    return false;
  }

  commander_->GetRobotVersion(robot_version_);
  std::cout << robot_version_.c_str() << std::endl;

  commander_->getPermissions();
  commander_->setLogLevel(LogLevels::SetCommand);

  commander_->setRobotMode(ControlMode::Auto);
  commander_->setPtpSpeed(100);
  commander_->setOverrideRatio(100);

  commander_->getActualPosition(prev_target_joint_positions_);

  servoAmpState_ = true;
  commander_->setServoAmpState(servoAmpState_);

  return true;
}

void HIWINDriver::disconnect()
{
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
  commander_->getServoAmpState(servoAmpState_);
  return servoAmpState_;
}

bool HIWINDriver::isMotionPossible()
{
  if (!servoAmpState_ || error_list_.size() || !commander_->isRemoteMode())
  {
    return false;
  }
  return true;
}

bool HIWINDriver::isInMotion()
{
  commander_->getMotionState(robotStatus_);
  if (robotStatus_ == MotionStatus::Moving)
  {
    return true;
  }
  return false;
}

bool HIWINDriver::isInError()
{
  commander_->getErrorCode(error_list_);
  if (error_list_.empty())
  {
    return false;
  }
  return true;
}

void HIWINDriver::getErrorCode(int32_t& error_code)
{
  if (error_list_.empty())
  {
    error_code = 0;
    return;
  }

  std::string last_error = error_list_.back();

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
  if (velocities.size() != 6 && velocities.size() > 9)
  {
    return;
  }

  double value[6];
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
    double ext_value[3];
    if (commander_->getExtActualRPM(ext_value) != 0)
    {
      return;
    }
    for (size_t i = 0; i < 3; i++)
    {
      velocities.at(6 + i) = ext_value[i];
    }
  }

  return;
}

void HIWINDriver::getJointEffort(std::vector<double>& efforts)
{
  double value[6];
  if (efforts.size() < 6)
  {
    return;
  }

  if (commander_->getActualCurrent(value) != 0)
  {
    return;
  }
  for (size_t i = 0; i < 6; i++)
  {
    efforts.at(i) = value[i];
  }

  return;
}

void HIWINDriver::getJointPosition(std::vector<double>& positions)
{
  if (positions.size() != 6 && positions.size() > 9)
  {
    return;
  }

  double value[6];
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
    double ext_value[3];
    if (commander_->getExtActualPosition(ext_value) != 0)
    {
      return;
    }
    for (size_t i = 0; i < 3; i++)
    {
      positions.at(6 + i) = ext_value[i];
    }
  }

  return;
}

void HIWINDriver::writeJointCommand(const std::vector<double>& positions, const float goal_time)
{
  if (positions.size() != 6 && positions.size() > 9)
  {
    return;
  }

  double* value = new double[positions.size()]();

  /*
   * goal_time
   * ... TBD.
   */

  std::copy(positions.begin(), positions.end(), value);

  if (positions.size() == 6)
  {
    commander_->ptpJoint(value, 100);
  }
  else if (positions.size() == 9)
  {
    commander_->extPtpJoint(value);
  }

  delete[] value;
}

void HIWINDriver::motionAbort()
{
  commander_->motionAbort();
}

}  // namespace hrsdk
