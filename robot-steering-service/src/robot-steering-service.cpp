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
  if (cid != 111) {
    std::cout << cid << " is not an valid conference ID number." << std::endl;
    return 0;
  }

  bool const verbose = (cmd.count("verbose") != 0);
  if (verbose) {
    std::cout << "Starting microservice." << std::endl;
  }

  cluon::OD4Session od4(cid);

  auto onGroundSpeedRequest{[&od4, &verbose](cluon::data::Envelope &&envelope) {
    auto const ppr =
        cluon::extractMessage<opendlv::proxy::GroundSpeedRequest>(
            std::move(envelope));

    float gSpeed = ppr.groundSpeed();
    if (verbose) {
      std::cout << "Got ground speed " << gSpeed << std::endl;
    }

    float mNumber = 2.5;
    float newSpeed = gSpeed*mNumber;
    if (verbose) {
      std::cout << "New multiplied speed " << newSpeed << std::endl;
    }
    
    opendlv::proxy::GroundSpeedReading speedSend;
    speedSend.groundSpeed(newSpeed);

    od4.send(speedSend);

    /////////////////Steering Algorithm Part/////////////////////////
    // Follow part might be dealed with another microservice later
    // Find the first and the last point and find the final aim point
    // cv::Point2f yellowConeFpt, yellowConeLpt, blueConeFpt, blueConeLpt;
    // cv::Point2f tempFpt, tempLpt, AimPt, CenterPt;
    // if ( usableYellowCones.size() > 0 && usableBlueCones.size() > 0 )
    // {
    //   yellowConeFpt = cv::Point2f(usableYellowCones[0].x, usableYellowCones[0].y + hsv.rows / 2);
    //   yellowConeLpt = cv::Point2f(usableYellowCones[usableYellowCones.size()-1].x, usableYellowCones[usableYellowCones.size()-1].y + hsv.rows / 2);
    //   blueConeFpt = cv::Point2f(usableBlueCones[0].x, usableBlueCones[0].y + hsv.rows / 2);
    //   blueConeLpt = cv::Point2f(usableBlueCones[usableBlueCones.size()-1].x, usableBlueCones[usableBlueCones.size()-1].y + hsv.rows / 2);
    // }
    // else if ( usableYellowCones.size() > 0 && usableBlueCones.size() == 0 )
    // {
    //   yellowConeFpt = cv::Point2f(usableYellowCones[0].x, usableYellowCones[0].y + hsv.rows / 2);
    //   yellowConeLpt = cv::Point2f(usableYellowCones[usableYellowCones.size()-1].x, usableYellowCones[usableYellowCones.size()-1].y + hsv.rows / 2);
    //   blueConeFpt = cv::Point2f(hsv.cols, hsv.rows);
    //   blueConeLpt = cv::Point2f(hsv.cols / 2, hsv.rows);
    // }
    // else if( usableYellowCones.size() == 0 && usableBlueCones.size() > 0 )
    // {  
    //   yellowConeFpt = cv::Point2f(0, hsv.rows);
    //   yellowConeLpt = cv::Point2f(hsv.cols / 2, hsv.rows);
    //   blueConeFpt = cv::Point2f(usableBlueCones[0].x, usableBlueCones[0].y + hsv.rows / 2);
    //   blueConeLpt = cv::Point2f(usableBlueCones[usableBlueCones.size()-1].x, usableBlueCones[usableBlueCones.size()-1].y + hsv.rows / 2);   
    // }
    // tempFpt = cv::Point2f((yellowConeFpt.x + blueConeFpt.x) / 2, (yellowConeFpt.y + blueConeFpt.y) / 2);
    // tempLpt = cv::Point2f((yellowConeLpt.x + blueConeLpt.x) / 2, (yellowConeLpt.y + blueConeLpt.y) / 2);
    // if ( usableYellowCones.size() == 0 )
    // {
    //   tempFpt = cv::Point2f((yellowConeFpt.x + tempFpt.x) / 2, (yellowConeFpt.y + tempFpt.y) / 2);
    //   tempLpt = cv::Point2f((yellowConeLpt.x + tempLpt.x) / 2, (yellowConeLpt.y + tempLpt.y) / 2);
    // }
    // AimPt = cv::Point2f((tempFpt.x + tempLpt.x) / 2, (tempFpt.y + tempLpt.y) / 2);
    // CenterPt = cv::Point2f(hsv.cols / 2, hsv.rows); 

    ////////////////////////////////////////////////////////////////
    // Do something with the distance readings if wanted.
    // {
    //   std::lock_guard<std::mutex> lck(distancesMutex);
    //   // std::cout << "front = " << front << ", "
    //   //           << "rear = " << rear << ", "
    //   //           << "left = " << left << ", "
    //   //           << "right = " << right << "." << std::endl;
    // }

    ////////////////////////////////////////////////////////////////
    // Example for creating and sending a message to other microservices;
    // can be removed when not needed.
    // opendlv::proxy::AngleReading ar;
    // ar.angle(123.45f);
    // od4.send(ar);

    ////////////////////////////////////////////////////////////////
    // Steering and acceleration/decelration.
    //
    // Uncomment the following lines to steer; range: +38deg (left) ..
    // -38deg (right). Value groundSteeringRequest.groundSteering must be
    // given in radians (DEG/180. * PI).

    // Calculate angle from detected cones
    // float Angle;
    // Angle = std::atan2(CenterPt.x - AimPt.x, CenterPt.y - AimPt.y);
    // // Angle = Angle * 38 / 180; // Normalize to 38 degree
    // Angle = Angle * 38 / 90; // Normalize to 38 degree
    // opendlv::proxy::GroundSteeringRequest gsr;
    // gsr.groundSteering(Angle);
    // od4.send(gsr);

    // // Uncomment the following lines to accelerate/decelerate; range: +0.25
    // // (forward) .. -1.0 (backwards). Be careful!
    // opendlv::proxy::PedalPositionRequest ppr;
    // ppr.position(0);
    // od4.send(ppr);  
  }};

  od4.dataTrigger(
      opendlv::proxy::GroundSpeedRequest::ID(), onGroundSpeedRequest);

  // float nTest = 1.0;
  while (od4.isRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Test Code
    // opendlv::proxy::GroundSpeedRequest speedRequest;
    // speedRequest.groundSpeed(nTest);
    // od4.send(speedRequest);    
    // nTest++;
  }

  if (verbose) {
    std::cout << "Closing microservice." << std::endl;
  }

  return 0;
}
