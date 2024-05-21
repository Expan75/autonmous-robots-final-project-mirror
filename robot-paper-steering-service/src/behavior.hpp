/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BEHAVIOR
#define BEHAVIOR

#include <mutex>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "opendlv-message-standard.hpp"
#include <iostream>
//#include "opendlv-standard-message-set.hpp"

class Behavior {
 private:
  Behavior(Behavior const &) = delete;
  Behavior(Behavior &&) = delete;
  Behavior &operator=(Behavior const &) = delete;
  Behavior &operator=(Behavior &&) = delete;

 public:
  Behavior() noexcept;
  ~Behavior() = default;

 public:
  opendlv::proxy::GroundSteeringRequest getGroundSteeringAngle() noexcept;
  opendlv::proxy::PedalPositionRequest getPedalPositionRequest() noexcept;
  void setFrontUltrasonic(opendlv::proxy::DistanceReading const &) noexcept;
  void setRearUltrasonic(opendlv::proxy::DistanceReading const &) noexcept;
  void setLeftIr(opendlv::proxy::DistanceReading const &) noexcept;
  void setRightIr(opendlv::proxy::DistanceReading const &) noexcept;
  void step(float maxPedalPosition, float minPedalPosition, float maxAngle, float minFrontDist, float minRearDist, float minLeftDist, float minRightDist) noexcept;
  void setGreenPaper(opendlv::logic::perception::DetectionProperty const &) noexcept;
  void setBluePaper(opendlv::logic::perception::DetectionProperty const &) noexcept;
  int32_t state;

 private:
  opendlv::proxy::DistanceReading m_frontUltrasonicReading;
  opendlv::proxy::DistanceReading m_rearUltrasonicReading;
  opendlv::proxy::DistanceReading m_leftIrReading;
  opendlv::proxy::DistanceReading m_rightIrReading;
  opendlv::proxy::GroundSteeringRequest m_groundSteeringAngleRequest;
  opendlv::proxy::PedalPositionRequest m_pedalPositionRequest;
  std::mutex m_frontUltrasonicReadingMutex;
  std::mutex m_rearUltrasonicReadingMutex;
  std::mutex m_leftIrReadingMutex;
  std::mutex m_rightIrReadingMutex;
  std::mutex m_groundSteeringAngleRequestMutex;
  std::mutex m_pedalPositionRequestMutex;
  int32_t m_newGreenPaper;
  int32_t m_newBluePaper;
  //int32_t state;
  float pedal;
  float steering;
  int32_t noGreenPaper;
  int32_t noBluePaper;
  std::chrono::steady_clock::time_point oldTime;
  int32_t wiggles;
  std::chrono::steady_clock::time_point lastGreen;
  opendlv::logic::perception::DetectionProperty m_bluePaper;
  opendlv::logic::perception::DetectionProperty m_greenPaper;
  std::mutex m_bluePaperMutex;
  std::mutex m_greenPaperMutex;
};

#endif
