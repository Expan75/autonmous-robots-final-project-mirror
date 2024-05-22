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
  float minPedalPosition = (cmd.count("minPed") != 0) ? std::stof(cmd["minPed"]) : 0.5f;
  float maxPedalBWPosition = (cmd.count("maxPedBW") != 0) ? std::stof(cmd["maxPedBW"]) : 1.5f;
  float minPedalBWPosition = (cmd.count("minPedBW") != 0) ? std::stof(cmd["minPedBW"]) : 1.0f;
  int maxSteering = (cmd.count("maxSter") != 0) ? std::stoi(cmd["maxSter"]) : 35;
  float maxAngle = maxSteering / static_cast<float>(180)* static_cast<float>(2*acos(0.0));
  float minFrontDist = (cmd.count("minFront") != 0) ? std::stof(cmd["minFront"]) : 0.2f;
  float minRearDist = (cmd.count("minRear") != 0) ? std::stof(cmd["minRear"]) : 0.2f;
  float minLeftDist = (cmd.count("minLeft") != 0) ? std::stof(cmd["minLeft"]) : 0.2f;
  float minRightDist = (cmd.count("minRight") != 0) ? std::stof(cmd["minRight"]) : 0.2f;
  float maxDist = (cmd.count("maxDist") != 0) ? std::stof(cmd["maxDist"]) : 300.0f;
  float minDist = (cmd.count("minDist") != 0) ? std::stof(cmd["minDist"]) : 150.0f;
  int avoidCount = (cmd.count("avoidCount") != 0) ? std::stoi(cmd["avoidCount"]) : 10;
  bool isCollisionAvoidanceMode = (cmd.count("Col") != 0) ? std::stoi(cmd["Col"]) : true;
  int BFCount = (cmd.count("BFCount") != 0) ? std::stoi(cmd["BFCount"]) : 30;
  int FBCount = (cmd.count("FBCount") != 0) ? std::stoi(cmd["FBCount"]) : 30;

  cluon::OD4Session od4(cid);

  // Handler to receive distance readings (realized as C++ lambda).
  std::mutex distancesMutex;
  float front{0};
  float rear{0};
  float left{0};
  float right{0};
  bool isDistEventTriggered = false;
  auto onDistance = [&distancesMutex, &front, &rear, &left, &right, &isDistEventTriggered](
                        cluon::data::Envelope &&env) {
    auto senderStamp = env.senderStamp();
    // Now, we unpack the cluon::data::Envelope to get the desired
    // DistanceReading.
    opendlv::proxy::DistanceReading dr =
        cluon::extractMessage<opendlv::proxy::DistanceReading>(
            std::move(env));

    // Store distance readings.
    std::lock_guard<std::mutex> lck(distancesMutex);
    switch (senderStamp) {
    case 0:
      front = dr.distance();
      break;
    case 2:
      rear = dr.distance();
      break;
    case 1:
      left = dr.distance();
      break;
    case 3:
      right = dr.distance();
      break;
    }
    isDistEventTriggered = true;
  };
  // Finally, we register our lambda for the message identifier for
  // opendlv::proxy::DistanceReading.
  od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

  float dist_kiwi{-1.0f};
  float aimDirection_kiwi{0.0f};
  bool isDetectionEventTriggered = false;
  auto onDetectionPropertyReading{[&dist_kiwi, &aimDirection_kiwi, &isDetectionEventTriggered](
      cluon::data::Envelope &&envelope)
  {
    auto DPReading = 
      cluon::extractMessage<opendlv::logic::perception::DetectionProperty>(
          std::move(envelope));

    std::string strPosition = DPReading.property();
    size_t comma_index = strPosition.find(";");
    if ( DPReading.detectionId() == 0 ){ // circle
      dist_kiwi = std::stof(strPosition.substr(0,comma_index));
      aimDirection_kiwi = std::stof(strPosition.substr(comma_index+1,strPosition.length()-1));
    }
    isDetectionEventTriggered = true;
  }};
  od4.dataTrigger(opendlv::logic::perception::DetectionProperty::ID(),
    onDetectionPropertyReading);

  float pedal{minPedalPosition}, steering{0.0f};
  int nFrontGoBackCount = 1;
  int nBackGoFrontCount = 1;
  int nAvoidCollisionCount = 0;
  while (od4.isRunning())
  {
    if ( isDetectionEventTriggered || isDistEventTriggered ){
      if( isDistEventTriggered ){
        isDistEventTriggered = false;
      }
      else if ( isDetectionEventTriggered ){
        isDetectionEventTriggered = false;
      }
    }
    else{
      continue;
    }

    // Create message instances
    opendlv::proxy::GroundSteeringRequest gsr;
    opendlv::proxy::PedalPositionRequest ppr;

    // Collision avoidance    
    if ( isCollisionAvoidanceMode &&( left < minLeftDist || right < minRightDist || front < minFrontDist || rear < minRearDist) ){
      // Set avoid collision count to 1
      nAvoidCollisionCount = 1;

      // Too close to front
      if ( front < minFrontDist ){
        if( left < minLeftDist ){
          steering = maxAngle; // Turn left
          pedal = -maxPedalBWPosition / 8.0f * 6.0f; // Go backward
        }
        else if( right < minRightDist ){
          steering = -maxAngle; // Turn right
          pedal = -maxPedalBWPosition / 8.0f * 6.0f; // Go backward
        }
        else{
          if ( nFrontGoBackCount > 0 ){
            steering = maxAngle; // Turn left
            pedal = -maxPedalBWPosition / 8.0f * 6.0f; // Go backward
            nFrontGoBackCount++;
          }
          else if ( nFrontGoBackCount < 0 ){
            steering = -maxAngle; // Turn right
            pedal = -maxPedalBWPosition / 8.0f * 6.0f; // Go backward
            nFrontGoBackCount--;
          }

          if ( nFrontGoBackCount > FBCount ){
            nFrontGoBackCount = 0;
            nFrontGoBackCount--;
          }
          else if ( nFrontGoBackCount < -FBCount ){
            nFrontGoBackCount = 0;
            nFrontGoBackCount++;
          }
        }

        // Write to other microservice
        ppr.position(pedal);
        od4.send(ppr);
        gsr.groundSteering(steering);
        od4.send(gsr);
        continue;
      }

      // Too close to back
      if ( rear < minRearDist ){
        if( left < minLeftDist || right < minRightDist ){
          steering = 0.0f; // No turn
          pedal = maxPedalPosition / 8.0f * 7.0f; // Go front
        }
        else{
          if ( nBackGoFrontCount > 0 ){
            steering = maxAngle; // Turn left
            pedal = maxPedalPosition / 8.0f * 7.0f; // Go front
            nBackGoFrontCount++;
          }
          else if ( nBackGoFrontCount < 0 ){
            steering = -maxAngle; // Turn right
            pedal = maxPedalPosition / 8.0f * 7.0f; // Go front
            nBackGoFrontCount--;
          }

          if ( nBackGoFrontCount > BFCount ){
            nBackGoFrontCount = 0;
            nBackGoFrontCount--;
          }
          else if ( nBackGoFrontCount < -BFCount ){
            nBackGoFrontCount = 0;
            nBackGoFrontCount++;
          }
        }

        // Write to other microservice
        ppr.position(pedal);
        od4.send(ppr);
        gsr.groundSteering(steering);
        od4.send(gsr);
        continue;
      }

      // Too close to left or right
      if ( left < minLeftDist && right < minRightDist ){ // Too close to left and right at the same time: no steering
        steering = 0.0f;
      }    
      else if ( left < minLeftDist ){ // Too close to left: turn right(not yet known, set to 0.2)
        steering = - maxAngle;
      }        
      else if ( right < minRightDist ){ // Too close to right: turn left
        steering =  maxAngle;
      }

      // Write to other microservice
      ppr.position(pedal);
      od4.send(ppr);
      gsr.groundSteering(steering);
      od4.send(gsr);
      continue;
    }

    // If avoid count is larger than 1, than continue to do the action
    if ( nAvoidCollisionCount > 0 && nAvoidCollisionCount < avoidCount ){
      // Do nothing with steer / pedal
      nAvoidCollisionCount++;
      continue;
    }  
    else if( nAvoidCollisionCount >= avoidCount ){
      nAvoidCollisionCount = 0; // Stop the avoid collision action after 10 count
    }
    
    // Follow kiwi car part
    if ( dist_kiwi < 0 ){ // Start to find around for kiwi car
      // std::cout << "Start to find blue papper" << std::endl;
      steering = 0.0f; // No turn
      pedal = maxPedalPosition * 0.6f; // Go front
    }
    else if ( dist_kiwi < minDist ){
      // Reach the blue papper, stop the car
      steering = aimDirection_kiwi / 90 * maxSteering;
      if ( steering >= maxAngle ){
        steering = maxAngle ;
      }
      else if ( steering <= -maxAngle ){
        steering = -maxAngle;
      }
      
      pedal = (minDist - dist_kiwi) / (maxDist - minDist) * (maxPedalBWPosition - minPedalBWPosition) + minPedalBWPosition; // Go front
      pedal = - pedal;
      if ( std::fabs(pedal) >= maxPedalBWPosition ){
        pedal = - maxPedalBWPosition;
      }
      std::cout << "Find kiwi car! the distance is :" << dist_kiwi << std::endl;
    }
    else{ // If blue papper exist, 
      // Setup steering
      std::cout << "kiwi car exist! try to find it. The dist is: " << dist_kiwi << std::endl;
      steering = aimDirection_kiwi / 90 * maxSteering;
      if ( steering >= maxAngle ){
        steering = maxAngle ;
      }
      else if ( steering <= -maxAngle ){
        steering = -maxAngle;
      }

      // Setup pedal
      pedal = (dist_kiwi - minDist) / (maxDist - minDist) * (maxPedalPosition - minPedalPosition) + minPedalPosition; // Go front
      if ( pedal >= maxPedalPosition ){
        pedal = maxPedalPosition;
      }
    }

    // Write to other microservice
    ppr.position(pedal);
    od4.send(ppr);
    gsr.groundSteering(steering);
    od4.send(gsr);
  } 

  return 0;
}
