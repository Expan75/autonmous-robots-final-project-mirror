#include <cmath>
#include <cstdint>
#include <iostream>

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"

int32_t main(int32_t argc, char **argv)
{
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if (cmd.count("cid") == 0) {
    std::cout << argv[0] << " is an OpenDLV microservice." << std::endl;
    std::cout << "Usage: " << argv[0] << " "
              << "--cid=<conference id; e.g. 111> "
              << "[--verbose] " << std::endl;
    return 0;
  }

  uint16_t const cid = std::stoi(cmd.at("cid"));
  // bool const verbose = (cmd.count("verbose") != 0);
  if (cid != 111) {
    std::cout << cid << " is not an valid conference ID number." << std::endl;
    return 0;
  }  
  float maxPedalPosition = (cmd.count("maxPed") != 0) ? std::stof(cmd["maxPed"]) : 1.0f;
  float minPedalPosition = (cmd.count("minPed") != 0) ? std::stof(cmd["minPed"]) : 0.6f;
  int maxSteering = (cmd.count("maxSter") != 0) ? std::stoi(cmd["maxSter"]) : 38;
  float maxAngle = maxSteering / static_cast<float>(180)* static_cast<float>(2*acos(0.0));

  cluon::OD4Session od4(cid);

  float aimDirection_cur{0.0f}, aimDirection_dev{0.0f}, steering{0.0f};
  bool isMessageReceived = false;
  auto onAimDirectionReading{[&aimDirection_cur, &aimDirection_dev, &steering
  , &maxAngle, &isMessageReceived](
      cluon::data::Envelope &&envelope)
    {
      auto ADReading = 
        cluon::extractMessage<opendlv::logic::action::AimDirection>(
            std::move(envelope));
      float aimDirection_new = ADReading.azimuthAngle();
      aimDirection_dev = (aimDirection_new - aimDirection_cur);
      aimDirection_cur = aimDirection_new;
      isMessageReceived = true;
    }};
  od4.dataTrigger(opendlv::logic::action::AimDirection::ID(),
    onAimDirectionReading);

  float pedal{minPedalPosition};
  while (od4.isRunning())
  {
    if( isMessageReceived == false ){
      continue;;
    }
    else{
      isMessageReceived = false;
    }

    // Create message instances
    opendlv::proxy::GroundSteeringRequest gsr;
    opendlv::proxy::PedalPositionRequest ppr;

    steering = steering + aimDirection_dev / static_cast<float>(acos(0.0)) * maxAngle;
    if ( steering >= maxAngle ){
      steering = maxAngle ;
    }
    else if ( steering <= -maxAngle ){
      steering = -maxAngle;
    }
    std::cout << "Steering: " << steering << std::endl;
    std::cout << "Aim Direction dev: " << aimDirection_dev << std::endl;
    std::cout << "Aim Direction current: " << aimDirection_cur << std::endl;

    // Calculate pedal position
    // speed up if the deviation of the angle is not too large
    if ( std::fabs(aimDirection_dev) < 0.2 ){
      if ( std::fabs(aimDirection_cur) < 0.15 ){
      // Speed up the pedal position
        pedal += 0.05f;
        if ( pedal >= maxPedalPosition ){
          pedal = maxPedalPosition;
        }
      }
      else{
        // Speed up the pedal position a bit
        pedal += 0.01f;
        if ( pedal >= (maxPedalPosition - minPedalPosition) * 0.6f + minPedalPosition ){
          pedal = (maxPedalPosition - minPedalPosition) * 0.6f + minPedalPosition ;
        }
      }
    }    
    else if ( std::fabs(aimDirection_dev) < 0.5 ){ // Slow down abit while the deviation is a bit large
      // slow down the pedal position
      pedal -= 0.03f;
      if ( pedal <= minPedalPosition ){
        pedal = minPedalPosition;
      }
    } 
    else{ // Slow down while the deviation is large
      // slow down the pedal position
      pedal -= 0.05f;
      if ( pedal <= minPedalPosition ){
        pedal = minPedalPosition;
      }
    }

    ppr.position(pedal);
    od4.send(ppr);
    gsr.groundSteering(steering);
    od4.send(gsr);
  } 

  return 0;
}
