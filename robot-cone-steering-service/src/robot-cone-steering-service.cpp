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
  int maxSteering = (cmd.count("maxSter") != 0) ? std::stoi(cmd["maxSter"]) : 35;

  cluon::OD4Session od4(cid);

  float aimDirection_cur{0.0f};
  float aimDirection_dev{0.0f};
  auto onAimDirectionReading{[&aimDirection_cur, &aimDirection_dev](
      cluon::data::Envelope &&envelope)
    {
      auto ADReading = 
        cluon::extractMessage<opendlv::logic::action::AimDirection>(
            std::move(envelope));
      float aimDirection_new = ADReading.azimuthAngle();
      aimDirection_dev = (aimDirection_new - aimDirection_cur);
      aimDirection_cur = aimDirection_new;
    }};
  od4.dataTrigger(opendlv::logic::action::AimDirection::ID(),
    onAimDirectionReading);

  float pedal{minPedalPosition}, steering{0.0f};
  while (od4.isRunning())
  {
    // Create message instances
    opendlv::proxy::GroundSteeringRequest gsr;
    opendlv::proxy::PedalPositionRequest ppr;

    // Calculate steering
    steering = steering + aimDirection_dev / 180 * maxSteering;
    if ( steering >= maxSteering / 180 * 2*acos(0.0) ){
      steering = maxSteering;
    }
    else if ( steering <= -maxSteering / 180 * 2*acos(0.0) ){
      steering = -maxSteering;
    }

    // Calculate pedal position
    // Stay in straight line
    if ( std::fabs(aimDirection_cur) < 0.1 ){ // The car is originally in straight line already
      if ( std::fabs(aimDirection_dev) < 0.1 ){ // If the aim direction change was small
        // Speed up the pedal position and do not steer
        pedal += 0.10f;
        if ( pedal >= maxPedalPosition ){
          pedal = maxPedalPosition;
        }
        ppr.position(pedal);
        od4.send(ppr);
        continue;
      }

      if ( std::fabs(aimDirection_dev) < 0.5 ){ // If the aim direction change was rather large
        // Speed up the pedal position slightly and do steer
        pedal += 0.05f;
        if ( pedal >= maxPedalPosition ){
          pedal = maxPedalPosition;
        }
        ppr.position(pedal);
        od4.send(ppr);
        gsr.groundSteering(steering);
        od4.send(gsr);
        continue;
      }

      // If the aim direction change is very large (larger than 30 degree)
      // slow down the pedal position
      pedal -= 0.01f;
      if ( pedal <= minPedalPosition ){
        pedal = minPedalPosition;
      }
      ppr.position(pedal);
      od4.send(ppr);
      gsr.groundSteering(steering);
      od4.send(gsr);
      continue;      
    }

    // If the direction doesn't change too much
    if ( std::fabs(aimDirection_dev) < 0.1 ){
      // Speed up the pedal position slightly and do steer
      pedal += 0.05f;
      if ( pedal >= maxPedalPosition ){
        pedal = maxPedalPosition;
      }
      ppr.position(pedal);
      od4.send(ppr);
      gsr.groundSteering(steering);
      od4.send(gsr);
      continue;
    }

    // If the direction change a bit
    if ( std::fabs(aimDirection_dev) < 0.5 ){
      // the aim direction stay in the same sign, speed up a bit then steer
      if ( (aimDirection_cur > 0 &&  aimDirection_cur - aimDirection_dev > 0)
        || (aimDirection_cur < 0 &&  aimDirection_cur - aimDirection_dev < 0) ){
        pedal += 0.01f;
        if ( pedal >= maxPedalPosition ){
          pedal = maxPedalPosition;
        }
      }
      else{
        pedal -= 0.01f;
        if ( pedal <= minPedalPosition ){
          pedal = minPedalPosition;
        }
      }
      ppr.position(pedal);
      od4.send(ppr);
      gsr.groundSteering(steering);
      od4.send(gsr);
      continue;
    }

    // If the direction change a lot
    // Slow down the pedal position and do steer
    pedal -= 0.05f;
    if ( pedal <= minPedalPosition ){
      pedal = minPedalPosition;
    }
    ppr.position(pedal);
    od4.send(ppr);
    gsr.groundSteering(steering);
    od4.send(gsr);
  } 

  return 0;
}
