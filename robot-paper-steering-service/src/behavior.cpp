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

#include "behavior.hpp"
Behavior::Behavior() noexcept:
  m_frontUltrasonicReading{},
  m_rearUltrasonicReading{},
  m_leftIrReading{},
  m_rightIrReading{},
  m_groundSteeringAngleRequest{},
  m_pedalPositionRequest{},
  m_frontUltrasonicReadingMutex{},
  m_rearUltrasonicReadingMutex{},
  m_leftIrReadingMutex{},
  m_rightIrReadingMutex{},
  m_groundSteeringAngleRequestMutex{},
  m_pedalPositionRequestMutex{},
  m_greenPaper{},
  m_bluePaper{},
  m_greenPaperMutex{},
  m_bluePaperMutex{}
{
  m_newBluePaper = 0;
  m_newGreenPaper = 0;
  state = 0;
  pedal = 0.0f;
  steering = 0.0f;
  noGreenPaper = 0;
  noBluePaper = 0;
  wiggles = 0;
  oldTime =std::chrono::steady_clock::now();
  lastGreen = std::chrono::steady_clock::now();
  }

opendlv::proxy::GroundSteeringRequest Behavior::getGroundSteeringAngle() noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleRequestMutex);
  return m_groundSteeringAngleRequest;
}

opendlv::proxy::PedalPositionRequest Behavior::getPedalPositionRequest() noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionRequestMutex);
  return m_pedalPositionRequest;
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::DistanceReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::DistanceReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}

void Behavior::setGreenPaper(opendlv::logic::perception::DetectionProperty const &greenPaper) noexcept
{
  std::lock_guard<std::mutex> lock(m_greenPaperMutex);
  m_greenPaper = greenPaper;
  m_newGreenPaper = 1;
}

void Behavior::setBluePaper(opendlv::logic::perception::DetectionProperty const &bluePaper) noexcept
{
  std::lock_guard<std::mutex> lock(m_bluePaperMutex);
  m_bluePaper = bluePaper;
  m_newBluePaper = 1;
  //std::cout<<"new blue paper"<<std::endl;
}

void Behavior::step(float maxPedalPosition, float minPedalPosition, float maxAngle, float minFrontDist, float minRearDist, float minLeftDist, float minRightDist) noexcept
{
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::DistanceReading leftIrReading;
  opendlv::proxy::DistanceReading rightIrReading;
  opendlv::logic::perception::DetectionProperty greenPaper;
  opendlv::logic::perception::DetectionProperty bluePaper;
  int32_t newBluePaper;
  int32_t newGreenPaper;
  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);
    std::lock_guard<std::mutex> lock5(m_greenPaperMutex);
    std::lock_guard<std::mutex> lock6(m_bluePaperMutex);
    
    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
    greenPaper = m_greenPaper;
    bluePaper = m_bluePaper;
    newGreenPaper = m_newGreenPaper;
    newBluePaper = m_newBluePaper;
  }

  float front = frontUltrasonicReading.distance();
  float rear = rearUltrasonicReading.distance();
  double left = leftIrReading.distance();
  double right = rightIrReading.distance();
  
  float xGreen{-1.0f};
  float yGreen{0.0f};
  if(newGreenPaper == 1)
  {
    std::string strPositionGreen = greenPaper.property();
    size_t comma_indexGreen = strPositionGreen.find(";");
    xGreen = std::stof(strPositionGreen.substr(0,comma_indexGreen));
    yGreen = std::stof(strPositionGreen.substr(comma_indexGreen+1,strPositionGreen.length()-1));
  }
  float xBlue{-1.0f};
  float yBlue{0.0f};
  if(newBluePaper == 1)
  {
    std::string strPositionBlue = bluePaper.property();
    size_t comma_indexBlue = strPositionBlue.find(";");
    xBlue = std::stof(strPositionBlue.substr(0,comma_indexBlue));
    yBlue = std::stof(strPositionBlue.substr(comma_indexBlue+1,strPositionBlue.length()-1));
  }

// If close to an object --> avoid it
  if( left < minLeftDist || right < minRightDist || front < minFrontDist || rear < minRearDist )
  { //std::cout << "avoid!" <<std::endl;
      // Steering
      if ( left < minLeftDist && right < minRightDist ){ // Too close to left and right at the same time: no steering
        steering = 0.0f;
      }    
      else if ( left < minLeftDist ){ // Too close to left: turn right(not yet known, set to 0.2)
        steering = -maxAngle;
      }        
      else if ( right < minRightDist ){ // Too close to right: turn left
        steering = maxAngle;
      }

      // Pedal
      if ( rear < minRearDist ){ // Too close to behind: go forward
        pedal = maxPedalPosition;
      }
      else if ( front < minFrontDist){ // Too close to foward : go backward
        pedal = -maxPedalPosition;
      }
  } else
  {
//////////////////////////////////////////////////  
    // Search for blue paper --> drive streight 
    if (state == 0)
    {
      if (newBluePaper == 1)
      {
        // if a new Blue paper was found --> change into second state to drive on it
        state = 1;
      } else 
      {
        // Drive streight
        steering = 0.0f;
        pedal = maxPedalPosition;
      }
    
    }
//////////////////////////////////////////////////
    // Drive towards blue paper
    if (state == 1) 
    {
      if(newBluePaper == 1)
      {
        noBluePaper = 0;
        // Calculate the angle need to turn and send it range: + 38 to - 38 degree
        cv::Point2f RefPt = cv::Point2f(640 / 2, 480);    
        steering = std::atan2(RefPt.x - xBlue, RefPt.y - yBlue);
        steering = steering * 38 / 90; // Normalize to 38 degree
      
        // Calculate the acc need to implement and send it range: +0.25 (forward) .. -1.0 (backwards)
        double dist = std::sqrt(std::pow(RefPt.x - xBlue, 2) + std::pow(RefPt.y - yBlue, 2));
        double farestDist = std::sqrt(std::pow(640, 2) + std::pow(480, 2));
        pedal = (float) (dist / farestDist) * 0.025f; // Normalize to 0.025(Use 1/10 of the max speed)
        if(pedal < minPedalPosition)
        {
          // We are on the paper --> start searching for the green one
          state = 2;
          // Start the timer, when to be back
          oldTime = std::chrono::steady_clock::now();
        }
      } else 
      {
        // The robot continous in the last known direction
        noBluePaper =+ 1;
        if(noBluePaper > 20)
        { //if no Blue paper found in the last cycles --> go back to searching
          state = 0; 
          steering = 0.0f;
          pedal = maxPedalPosition;
        }
      }
    } 
///////////////////////////////////////////////////  
    // Find the post it
    if (state == 2)
    {
      // Get time since been on blue paper the last time
      std::chrono::steady_clock::time_point newTime = std::chrono::steady_clock::now();
      std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(newTime - oldTime);

      // wait a little before steering on a new green paper in order to drive away from the old one
      std::chrono::duration<double> time_spanGreen = std::chrono::duration_cast<std::chrono::duration<double>>(newTime - lastGreen);
    
      if (time_span.count() > 120.0)
      {
        // Starts searching for blue paper after 180 sec. 
        state = 0;
        steering = 0.0f;
        pedal = maxPedalPosition;
      } else if (newGreenPaper == 1 && time_spanGreen.count() > 1.5)
      { 
        // if a new Green paper was found --> change into third state to drive on it
        // wait a little before steering on a new green paper in order to drive away from the old one
        state = 3;
      } else 
      {
        // Drive streight
        steering = 0.0f;
        pedal = maxPedalPosition;
      }
    }
///////////////////////////////////////////////////
    // Drive on the green paper  
    if (state == 3)
    {
      // Get time since been on blue paper the last time
      std::chrono::steady_clock::time_point newTime = std::chrono::steady_clock::now();
      std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(newTime - oldTime);
    
      if (time_span.count() > 120.0)
      {
        // Starts searching for blue paper after 180 sec. 
        state = 0;
        steering = 0.0f;
        pedal = maxPedalPosition;
      } else if(newGreenPaper == 1)
      {
        noGreenPaper = 0;
        // Calculate the angle need to turn and send it range: + 38 to - 38 degree
        cv::Point2f RefPt = cv::Point2f(640 / 2, 480);    
        steering = std::atan2(RefPt.x - xGreen, RefPt.y - yGreen);
        steering = steering * 38 / 90; // Normalize to 38 degree
      
        // Calculate the acc need to implement and send it range: +0.25 (forward) .. -1.0 (backwards)
        double dist = std::sqrt(std::pow(RefPt.x - xGreen, 2) + std::pow(RefPt.y - yGreen, 2));
        double farestDist = std::sqrt(std::pow(640, 2) + std::pow(480, 2));
        pedal = (float) (dist / farestDist) * 0.025f; // Normalize to 0.025(Use 1/10 of the max speed)
        if(pedal < minPedalPosition)
        {
          // We are on the post it --> start wiggeling
          state = 4;
          // Reset the conter for the wiggeling
          wiggles = 0;
        }
      } else 
      {
        // The robot continous in the last known direction
        noGreenPaper =+ 1;
        if(noGreenPaper > 20)
        { //if no Green paper found in the last cycles --> go back to searching
          state = 2; 
          steering = 0.0f;
          pedal = maxPedalPosition;
        }
      }
    }
////////////////////////////////////////////////////////
    // Wiggle the wheels
    if(state == 4)
    {
      // Get time since been on blue paper the last time
      std::chrono::steady_clock::time_point newTime = std::chrono::steady_clock::now();
      std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(newTime - oldTime);
      if (time_span.count() > 120.0)
      {
        // Starts searching for blue paper after 180 sec. 
        state = 0;
        steering = 0.0f;
        pedal = maxPedalPosition;
      } else 
      {
        if (wiggles ==40)
        {
          // End wiggling and seach new post it
          state = 2;
          steering = 0.0f;
          pedal = maxPedalPosition;
          lastGreen = newTime;
        } else if (wiggles < 10 || (wiggles >= 20 && wiggles < 30))
        {
          pedal = 0.0f;
          steering = maxAngle;
          wiggles += 1;;
        } else 
        {
          pedal = 0.0f;
          steering = -maxAngle;
          wiggles += 1;;
        }
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock1(m_groundSteeringAngleRequestMutex);
    std::lock_guard<std::mutex> lock2(m_pedalPositionRequestMutex);

    opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest;
    groundSteeringAngleRequest.groundSteering(steering);
    m_groundSteeringAngleRequest = groundSteeringAngleRequest;

    opendlv::proxy::PedalPositionRequest pedalPositionRequest;
    pedalPositionRequest.position(pedal);
    m_pedalPositionRequest = pedalPositionRequest;

    // Reset the variables for new Paper, that they are up to date
    std::lock_guard<std::mutex> lock3(m_bluePaperMutex);
    std::lock_guard<std::mutex> lock4(m_greenPaperMutex);
    m_newBluePaper = 0;
    m_newGreenPaper = 0;
  }
}
