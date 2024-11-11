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

void HIWINDriver::getJointVelocity(std::vector<double>& velocities)
{
  double value[6];
  if (velocities.size() < 6)
  {
    return;
  }

  if (commander_->getActualRPM(value) != 0)
  {
    return;
  }

  velocities.assign(value, value + 6);
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

  efforts.assign(value, value + 6);
  return;
}

void HIWINDriver::getJointPosition(std::vector<double>& positions)
{
  double value[6];
  if (positions.size() < 6)
  {
    return;
  }

  if (commander_->getActualPosition(value) != 0)
  {
    return;
  }

  positions.assign(value, value + 6);
  return;
}

void HIWINDriver::writeJointCommand(const std::vector<double>& positions, const float goal_time)
{
  double distance[6];
  double value[6];
  if (positions.size() != 6)
  {
    return;
  }

  for (size_t i = 0; i < 6; i++)
  {
    distance[i] = positions[i] - prev_target_joint_positions_[i];
    (distance[i] < 0) ? distance[i] *= -1 : distance[i] *= 1;
    prev_target_joint_positions_[i] = positions[i];
  }

  double max_distance = *std::max_element(distance, distance + 6);
  double velocity = max_distance / goal_time;

  /*
   * ... TBD.
   */

  std::copy(positions.begin(), positions.end(), value);

  commander_->ptpJoint(value, 100);
}

void HIWINDriver::motionAbort()
{
  commander_->motionAbort();
}

}  // namespace hrsdk
