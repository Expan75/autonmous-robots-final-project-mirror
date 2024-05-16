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
  bool const verbose = (cmd.count("verbose") != 0);
  if (cid != 111) {
    std::cout << cid << " is not an valid conference ID number." << std::endl;
    return 0;
  }  
  float maxPedalPosition = (cmd.count("maxPed") != 0) ? std::stof(cmd["maxPed"]) : 1.0f;
  float minPedalPosition = (cmd.count("minPed") != 0) ? std::stof(cmd["minPed"]) : 0.6f;
  int maxSteering = (cmd.count("maxSter") != 0) ? std::stoi(cmd["maxSter"]) : 35;

  cluon::OD4Session od4(cid);

  // Handler to receive distance readings (realized as C++ lambda).
  std::mutex distancesMutex;
  float front{0};
  float rear{0};
  float left{0};
  float right{0};
  auto onDistance = [&distancesMutex, &front, &rear, &left, &right](
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
  };
  // Finally, we register our lambda for the message identifier for
  // opendlv::proxy::DistanceReading.
  od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);

  float dist_b_papper{0.0f};
  float aimDirection_b_papper{0.0f};
  float dist_g_papper{0.0f};
  float aimDirection_g_papper{0.0f};
  auto onDetectionPropertyReading{[&dist_b_papper, &aimDirection_b_papper
  , &dist_g_papper, &aimDirection_g_papper](
      cluon::data::Envelope &&envelope)
    {
      auto DPReading = 
        cluon::extractMessage<opendlv::logic::perception::DetectionProperty>(
            std::move(envelope));

      std::string strPosition = DPReading.property();
      size_t comma_index = strPosition.find(";");
      if ( DPReading.detectionId() == 0 ){
        dist_g_papper = std::stof(strPosition.substr(0,comma_index));
        aimDirection_g_papper = std::stof(strPosition.substr(comma_index+1,strPosition.length()-1));
      }
      else if ( DPReading.detectionId() == 1 ){
        dist_b_papper = std::stof(strPosition.substr(0,comma_index));
        aimDirection_b_papper = std::stof(strPosition.substr(comma_index+1,strPosition.length()-1));
      }
    }};
  od4.dataTrigger(opendlv::logic::perception::DetectionProperty::ID(),
    onDetectionPropertyReading);

  float pedal{minPedalPosition}, steering{0.0f};
  while (od4.isRunning())
  {
    // Create message instances
    opendlv::proxy::GroundSteeringRequest gsr;
    opendlv::proxy::PedalPositionRequest ppr;

    // Collision avoidance    
    if ( left < 0.2f || right < 0.2f || front < 0.2 || rear < 0.2 ){
      // Steering
      if ( left < 0.2 && right < 0.2 ){ // Too close to left and right at the same time: no steering
        steering = 0.0f;
      }    
      else if ( left < 0.2 ){ // Too close to left: turn right(not yet known, set to 0.2)
        steering = -0.4f;
      }        
      else if ( right < 0.2 ){ // Too close to right: turn left
        steering = 0.4f;
      }

      // Pedal
      if ( rear < 0.2 ){ // Too close to behind: go forward
        pedal = 0.7f;
      }
      else if ( top < 0.2 ){ // Too close to foward : go backward
        pedal = -0.7f;
      }

      // Write to other microservice
      ppr.position(pedal);
      od4.send(ppr);
      gsr.groundSteering(steering);
      od4.send(gsr);
      continue;
    }

    // Without Aim Point
    // Search around(can we use area or green papper to point out the area that has been searched?)
    

    // With Aim Point
    // Blue Papper
    // Slow reach(first time)
    // Fast reach(others), maybe we can know about the position?

    // Green Papper
    // Fast reach

  } 

  return 0;
}
