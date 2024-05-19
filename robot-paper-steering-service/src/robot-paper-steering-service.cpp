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
  // float maxPedalPosition = (cmd.count("maxPed") != 0) ? std::stof(cmd["maxPed"]) : 1.0f;
  float minPedalPosition = (cmd.count("minPed") != 0) ? std::stof(cmd["minPed"]) : 0.6f;
  int maxSteering = (cmd.count("maxSter") != 0) ? std::stoi(cmd["maxSter"]) : 35;
  float maxAngle = maxSteering / static_cast<float>(180)* static_cast<float>(2*acos(0.0));
  float minFrontDist = (cmd.count("minFront") != 0) ? std::stof(cmd["minFront"]) : 0.2f;
  float minRearDist = (cmd.count("minRear") != 0) ? std::stof(cmd["minRear"]) : 0.2f;
  float minLeftDist = (cmd.count("minLeft") != 0) ? std::stof(cmd["minLeft"]) : 0.2f;
  float minRightDist = (cmd.count("minRight") != 0) ? std::stof(cmd["minRight"]) : 0.2f;
  float minBPDist = (cmd.count("minBP") != 0) ? std::stof(cmd["minBP"]) : 120.0f;
  float minGPDist = (cmd.count("minGP") != 0) ? std::stof(cmd["minGP"]) : 150.0f;

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
    // std::cout << "Distance front: " << front 
    //           << ", Distance rear: " << rear
    //           << ", Distance left: " << left
    //           << ", Distance right: " << right << std::endl;
    isDistEventTriggered = true;
  };
  // Finally, we register our lambda for the message identifier for
  // opendlv::proxy::DistanceReading.
  od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

  float dist_b_papper{-1.0f};
  float aimDirection_b_papper{0.0f};
  float dist_g_papper{-1.0f};
  float aimDirection_g_papper{0.0f};
  bool isDetectionEventTriggered = false;
  auto onDetectionPropertyReading{[&dist_b_papper, &aimDirection_b_papper
  , &dist_g_papper, &aimDirection_g_papper, &isDetectionEventTriggered](
      cluon::data::Envelope &&envelope)
  {
    auto DPReading = 
      cluon::extractMessage<opendlv::logic::perception::DetectionProperty>(
          std::move(envelope));

    std::string strPosition = DPReading.property();
    size_t comma_index = strPosition.find(";");
    if ( DPReading.detectionId() == 0 ){ // Green
      dist_g_papper = std::stof(strPosition.substr(0,comma_index));
      aimDirection_g_papper = std::stof(strPosition.substr(comma_index+1,strPosition.length()-1));
    }
    else if ( DPReading.detectionId() == 1 ){ // Blue
      dist_b_papper = std::stof(strPosition.substr(0,comma_index));
      aimDirection_b_papper = std::stof(strPosition.substr(comma_index+1,strPosition.length()-1));
    }
    isDetectionEventTriggered = true;
    // std::cout << "Distance to blue papper: " << dist_b_papper 
    //           << "Angle to blue papper: " << aimDirection_b_papper << std::endl;
    // std::cout << "Distance to green papper: " << dist_g_papper 
    //           << "Angle to green papper: " << aimDirection_g_papper << std::endl;
  }};
  od4.dataTrigger(opendlv::logic::perception::DetectionProperty::ID(),
    onDetectionPropertyReading);

  float pedal{minPedalPosition}, steering{0.0f};
  int nFrontGoBackCount = 0;
  int nBackGoFrontCount = 0;
  int nAvoidCollisionCount = 0;
  int nStartToFindBluePapperCount = 3*1e3;
  int nWiggleCount = 0;
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

    // Add count to find blue papper
    nStartToFindBluePapperCount++;

    // Add count to wiggle count to drive away from current green papper
    if ( nWiggleCount >= 10 && nWiggleCount < 20 ){
      // std::cout << "Wiggle Count over 10" << std::endl;
      nWiggleCount++;
    }
    else if ( nWiggleCount >= 20 ){
      // std::cout << "Wiggle Count over 20, reset!" << std::endl;
      nWiggleCount = 0;
    }

    // Collision avoidance    
    if ( left < minLeftDist || right < minRightDist || front < minFrontDist || rear < minRearDist ){
      // Set avoid collision count to 1
      nAvoidCollisionCount = 1;

      // Too close to front
      if ( front < minFrontDist ){
        if( left < minLeftDist ){
          steering = 0.4f; // Turn left
          pedal = -0.6f; // Go backward
        }
        else if( right < minRightDist ){
          steering = -0.4f; // Turn right
          pedal = -0.6f; // Go backward
        }
        else{
          if ( nFrontGoBackCount > 0 ){
            steering = 0.4f; // Turn left
            pedal = -0.6f; // Go backward
            nFrontGoBackCount++;
          }
          else if ( nFrontGoBackCount < 0 ){
            steering = -0.4f; // Turn right
            pedal = -0.6f; // Go backward
            nFrontGoBackCount--;
          }

          if ( nFrontGoBackCount > 3 ){
            nFrontGoBackCount = 0;
            nFrontGoBackCount--;
          }
          else if ( nFrontGoBackCount < -3 ){
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
          pedal = 0.7f; // Go front
        }
        else{
          if ( nBackGoFrontCount > 0 ){
            steering = 0.4f; // Turn left
            pedal = 0.7f; // Go front
            nBackGoFrontCount++;
          }
          else if ( nBackGoFrontCount < 0 ){
            steering = -0.4f; // Turn right
            pedal = 0.7f; // Go front
            nBackGoFrontCount--;
          }

          if ( nBackGoFrontCount > 3 ){
            nBackGoFrontCount = 0;
            nBackGoFrontCount--;
          }
          else if ( nBackGoFrontCount < -3 ){
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
        steering = -0.2f;
      }        
      else if ( right < minRightDist ){ // Too close to right: turn left
        steering = 0.2f;
      }

      // Write to other microservice
      ppr.position(pedal);
      od4.send(ppr);
      gsr.groundSteering(steering);
      od4.send(gsr);
      continue;
    }

    // If avoid count is larger than 1, than continue to do the action
    if ( nAvoidCollisionCount > 0 && nAvoidCollisionCount < 10 ){
      // Do nothing with steer / pedal
      nAvoidCollisionCount++;
      continue;
    }  
    else if( nAvoidCollisionCount >= 10 ){
      nAvoidCollisionCount = 0; // Stop the avoid collision action after 10 count
    }
    
    // Blue Papper
    if ( nStartToFindBluePapperCount > 3*1e3 ){
      if ( dist_b_papper < 0 ){ // Start to find around for blue papper
        // std::cout << "Start to find blue papper" << std::endl;
        steering = 0.0f; // No turn
        pedal = 0.6f; // Go front
      }
      else if ( dist_b_papper < minBPDist ){
        // Reach the blue papper, stop the car
        steering = 0.0f; // No turn
        pedal = 0.0f; // Go front
        nStartToFindBluePapperCount = 0;
        std::cout << "Find blue papper! the distance is :" << dist_b_papper << std::endl;
      }
      else{ // If blue papper exist, 
        // Setup steering
        std::cout << "blue papper exist! try to find it. The dist is: " << dist_b_papper << std::endl;
        steering = aimDirection_b_papper / 90 * maxSteering;
        if ( steering >= maxAngle ){
          steering = maxAngle ;
        }
        else if ( steering <= -maxAngle ){
          steering = -maxAngle;
        }

        // Setup pedal
        pedal = 0.7f; // Go front
      }

      // Write to other microservice
      ppr.position(pedal);
      od4.send(ppr);
      gsr.groundSteering(steering);
      od4.send(gsr);
      continue;
    }

    // Try to find Green Papper
    if ( dist_g_papper < 0 || nWiggleCount >= 10 ){ // Start to find around for another green papper
        steering = 0.0f; // No turn
        pedal = 0.6f; // Go front
    }
    else if ( dist_g_papper < minGPDist && nWiggleCount < 10 ){
      // Reach the blue papper, stop the car and do wiggles
      pedal = 0.0f; // stop
      steering = 0.0f;
      std::cout << "Find green papper, start to wiggle!" << std::endl;

      // Do wiggle
      // std::this_thread::sleep_for(std::chrono::milliseconds(800)); // use delay to not let wiggle command sent to fast
      if ( steering >= 0.0f ){
        steering += 0.25f;    
      }
      else{
        steering -= 0.25f;
      }
        
      if ( steering > 0.5f ){
        steering = -0.25f ;
      }
      else if ( steering < -0.5f ){
        steering = 0.25f ;
      }
      nWiggleCount++;
    }
    else { // If green papper exist, 
      // Setup steering
        std::cout << "green papper exist! try to find it. The dist is: " << dist_g_papper << std::endl;
      steering = aimDirection_g_papper / 90 * maxSteering;
      if ( steering >= maxAngle ){
        steering = maxAngle ;
      }
      else if ( steering <= -maxAngle ){
        steering = -maxAngle;
      }

      // Setup pedal
      pedal = 0.7f; // Go front
    }

    // Write to other microservice
    ppr.position(pedal);
    od4.send(ppr);
    gsr.groundSteering(steering);
    od4.send(gsr);
  } 

  return 0;
}
