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

#ifndef HIWIN_ROBOT_CLIENT_LIBRARY_COMMANDER_HPP_
#define HIWIN_ROBOT_CLIENT_LIBRARY_COMMANDER_HPP_

#include <hiwin_robot_client_library/socket/tcp_client.hpp>

namespace hrsdk
{

enum class ControlMode : uint16_t
{
  Manual = 0,
  Auto
};

enum class MotionStatus : uint16_t
{
  ServerOff = 0,
  Waiting = 1,
  Running = 2,
  Hold = 3,
  Delay = 4,
  Moving = 5,
};

enum class LogLevels
{
  None = 0,
  Info,
  SetCommand,
  Console,
  Save,
};

class Commander : public socket::TCPClient
{
private:
  std::string robot_ip_;
  int port_;

public:
  Commander(const std::string& robot_ip, const int port);
  ~Commander();

  bool connect();
  bool isRemoteMode();

  int getPermissions();
  int setLogLevel(LogLevels level);
  int setServoAmpState(bool& enable);
  int getServoAmpState(bool& enable);

  int getActualRPM(double (&velocities)[6]);
  int getActualPosition(double (&positions)[6]);
  int getActualCurrent(double (&efforts)[6]);

  int getExtActualRPM(double (&velocities)[3]);
  int getExtActualPosition(double (&positions)[3]);

  int getMotionState(MotionStatus& status);
  int getErrorCode(std::vector<std::string>& error_list);

  int ptpJoint(double* positions, double ratio);
  int extPtpJoint(double* positions);

  int motionAbort();
  int setPtpSpeed(int ratio);
  int getPtpSpeed(int& ratio);
  int setOverrideRatio(int ratio);
  int getOverrideRatio(int& ratio);

  int setRobotMode(ControlMode mode);
  int getRobotMode(ControlMode& mode);
  int GetRobotVersion(std::string& str);
};

}  // namespace hrsdk

#endif  // HIWIN_ROBOT_CLIENT_LIBRARY_COMMANDER_HPP_
