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

#include "cluon-complete.hpp"
//#include "opendlv-standard-message-set.hpp"
#include "opendlv-message-standard.hpp"
#include "behavior.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  // Check that the mandatory command line arguments are present.
  if (0 == cmd.count("cid")
      || 0 == cmd.count("freq")) {
    std::cerr << argv[0] << "freq: frequency of the control, maxPed: driving speed, minPed: determines when the robot stopps on the paper, "
    << "maxSter: max. steering angle in deg., minXY: min. Distance to XY" << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --freq=<Integration frequency> "
      << "--cid=<OpenDaVINCI session> [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --freq=10 --cid=111" << std::endl;
    retCode = 1;
  } else {
    // Extract the command line arguments.
    bool const VERBOSE{cmd.count("verbose") != 0};
    uint16_t const CID = std::stoi(cmd["cid"]);
    float const FREQ = std::stof(cmd["freq"]); // std::stof(commandlineArguments["freq"]);
    float maxPedalPosition = (cmd.count("maxPed") != 0) ? std::stof(cmd["maxPed"]) : 1.0f;
    float minPedalPosition = (cmd.count("minPed") != 0) ? std::stof(cmd["minPed"]) : 0.6f;
    int maxSteering = (cmd.count("maxSter") != 0) ? std::stoi(cmd["maxSter"]) : 35;
    float maxAngle = maxSteering / static_cast<float>(180)* static_cast<float>(2*acos(0.0));
    float minFrontDist = (cmd.count("minFront") != 0) ? std::stof(cmd["minFront"]) : 0.2f;
    float minRearDist = (cmd.count("minRear") != 0) ? std::stof(cmd["minRear"]) : 0.2f;
    float minLeftDist = (cmd.count("minLeft") != 0) ? std::stof(cmd["minLeft"]) : 0.15f;
    float minRightDist = (cmd.count("minRight") != 0) ? std::stof(cmd["minRight"]) : 0.15f;
    // &maxPedalPosition, &minPedalPosition, &maxAngle, &minFrontDist, &minRearDist, &minLeftDist, &minRightDist
    // maxPedalPosition, minPedalPosition, maxAngle, minFrontDist, minRearDist, minLeftDist, minRightDist

    // The behaviour is put in its own class, and an object is created here.
    Behavior behavior;

    // The OD4 data trigger lambda functions.
    auto onDistanceReading{[&behavior](cluon::data::Envelope &&envelope)
      {
        auto distanceReading = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
        uint32_t const senderStamp = envelope.senderStamp();
        if (senderStamp == 0) {
          behavior.setFrontUltrasonic(distanceReading);
        } else if (senderStamp == 1) {
          behavior.setRearUltrasonic(distanceReading);
        } else if (senderStamp == 2) {
          behavior.setLeftIr(distanceReading);
        } else if (senderStamp == 3) {
          behavior.setRightIr(distanceReading);
        }
        //std::cout << "Distancereading"<< std::endl;
      }};
      auto onDetectionPropertyReading{[&behavior](cluon::data::Envelope &&envelope)
      {
        auto DPReading = 
          cluon::extractMessage<opendlv::logic::perception::DetectionProperty>(
            std::move(envelope));
        
        if ( DPReading.detectionId() == 0 ){
          behavior.setGreenPaper(DPReading);
        }
        else if ( DPReading.detectionId() == 1 ){
          behavior.setBluePaper(DPReading);
        }
        //std::cout << "Got DetectionPorpertyReading"<< std::endl;
      }};

    // Createing the OD4 session.
    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
    od4.dataTrigger(opendlv::logic::perception::DetectionProperty::ID(), onDetectionPropertyReading);

    // Lambda function to run at a specified frequency.
    auto atFrequency{[&VERBOSE, &behavior, &od4, &maxPedalPosition, &minPedalPosition, &maxAngle, &minFrontDist, &minRearDist, &minLeftDist, &minRightDist]() -> bool
      {
        behavior.step(maxPedalPosition, minPedalPosition, maxAngle, minFrontDist, minRearDist, minLeftDist, minRightDist);
        auto groundSteeringAngleRequest = behavior.getGroundSteeringAngle();
        auto pedalPositionRequest = behavior.getPedalPositionRequest();

        cluon::data::TimeStamp sampleTime = cluon::time::now();
        od4.send(groundSteeringAngleRequest, sampleTime, 0);
        od4.send(pedalPositionRequest, sampleTime, 0);
        if (VERBOSE) {
          std::cout << "Ground steering angle is "
            << groundSteeringAngleRequest.groundSteering()
            << " and pedal position is " << pedalPositionRequest.position()
            << "; state: " << behavior.state
            << std::endl;
        }

        return true;
      }};

    // This will block until Ctrl+C is pressed.
    od4.timeTrigger(FREQ, atFrequency);
  }
  return retCode;
}
