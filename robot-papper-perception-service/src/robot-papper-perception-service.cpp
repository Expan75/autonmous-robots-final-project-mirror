/*
 * Copyright (C) 2024 OpenDLV
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if ((0 == cmd.count("cid")) || (0 == cmd.count("name")) ||
      (0 == cmd.count("width")) || (0 == cmd.count("height"))) {
    std::cout << argv[0]
              << " attaches to a shared memory area containing an ARGB image."
              << std::endl;
    std::cout << "Usage:   " << argv[0] << " "
              << "--cid=<OD4 session> --name=<name of shared memory area> "
                 "--width=<width of the video> --height=<height of the video> "
                 "[--verbose]"
              << std::endl;
    std::cout << "Example: " << argv[0] << " "
              << "--cid=112 --name=img.argb --width=640 --height=480 --verbose"
              << std::endl;
  } else {
    std::string const name{cmd["name"]};
    uint32_t const width{static_cast<uint32_t>(std::stoi(cmd["width"]))};
    uint32_t const height{static_cast<uint32_t>(std::stoi(cmd["height"]))};
    bool const verbose{cmd.count("verbose") != 0};

    // Attach to the shared memory.
    std::unique_ptr<cluon::SharedMemory> sharedMemory{
        new cluon::SharedMemory{name}};
    if (sharedMemory && sharedMemory->valid()) {
      std::clog << argv[0] << ": Attached to shared memory '"
                << sharedMemory->name() << " (" << sharedMemory->size()
                << " bytes)." << std::endl;

      // Interface to a running OD4 session; here, you can send and
      // receive messages.
      cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(cmd["cid"]))};
      
      // Try to catch the position of the object and send out the acceleration and the turning angle
      auto onDetectionProperty{[&od4, &verbose](cluon::data::Envelope &&envelope) {
        auto const Dp =
            cluon::extractMessage<opendlv::logic::perception::DetectionProperty>(
                std::move(envelope));

        std::string strPosition = Dp.property();
        if (verbose) {
          // std::cout << "Got object position " << strPosition << std::endl;
        }

        size_t comma_index = strPosition.find(";");
        float center_x = std::stof(strPosition.substr(0,comma_index));
        float center_y = std::stof(strPosition.substr(comma_index+1,strPosition.length()-1));

        // Calculate the angle need to turn and send it range: + 38 to - 38 degree
        cv::Point2f RefPt = cv::Point2f(640 / 2, 480);    
        float AngleDisplay;
        float Angle = std::atan2(RefPt.x - center_x, RefPt.y - center_y);
        AngleDisplay = Angle;
        Angle = Angle * 38 / 90; // Normalize to 38 degree
        opendlv::proxy::GroundSteeringRequest gsr;
        gsr.groundSteering(Angle);
        od4.send(gsr);

        // Calculate the acc need to implement and send it range: +0.25 (forward) .. -1.0 (backwards)
        double dist = std::sqrt(std::pow(RefPt.x - center_x, 2) + std::pow(RefPt.y - center_y, 2));
        double farestDist = std::sqrt(std::pow(640, 2) + std::pow(480, 2));
        double distDeviation = dist / farestDist * 0.025; // Normalize to 0.025(Use 1/10 of the max speed)
        opendlv::proxy::PedalPositionRequest ppr;
        ppr.position(static_cast<float>(distDeviation));
        od4.send(ppr); 
        // std::cout << "Got object position x:" << RefPt.x - center_x << " ,y: " << RefPt.y - center_y << std::endl;
        std::cout << "Angle Turn:" << AngleDisplay << " , Acc: " << distDeviation << std::endl;
      }};

      od4.dataTrigger(
      opendlv::logic::perception::DetectionProperty::ID(), onDetectionProperty);

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning()) {
        cv::Mat img;

        // Wait for a notification of a new frame.
        sharedMemory->wait();

        // Lock the shared memory.
        sharedMemory->lock();
        {
          // Copy image into cvMat structure.
          // Be aware of that any code between lock/unlock is blocking
          // the camera to provide the next frame. Thus, any
          // computationally heavy algorithms should be placed outside
          // lock/unlock
          cv::Mat wrapped(height, width, CV_8UC3, sharedMemory->data());
          img = wrapped.clone();
        }
        sharedMemory->unlock();

        // TODO: Do something with the frame.    
        // Sharpening the image first
        double alpha = 2; /*< Simple contrast control */
        double beta = 1; /*< Simple brightness control */
        cv::Mat sharpenImage;
        img.convertTo(sharpenImage, -1, alpha, beta); 
        // cv::imshow("Sharpen Image", sharpenImage);

        // Turn the color to HSV
        cv::Mat hsv;
        cv::cvtColor(sharpenImage, hsv, cv::COLOR_BGR2HSV);

        // Extract the blue papper
        cv::Mat bC_papper;
        cv::inRange(hsv, cv::Scalar(110,50,50), cv::Scalar(130,255,255), bC_papper);  
        cv::Mat element = getStructuringElement( cv::MORPH_RECT , cv::Size( 3,3 ), cv::Point( 0,0 ) );
        cv::morphologyEx( bC_papper, bC_papper, cv::MORPH_CLOSE , element );
        cv::dilate(bC_papper,bC_papper,element, cv::Point(-1,-1),3,1,1);
        
        // Find the contour of the extract papper image
        std::vector<std::vector<cv::Point>> contours_blue;
        cv::findContours(bC_papper, contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Iterate over all contours to find the blue papper
        std::vector<cv::Point> PapperContour;   
        std::vector<cv::Point> ContourCompare{cv::Point(55, 231), cv::Point(73, 376), cv::Point(152, 382), cv::Point(300, 340), cv::Point(286, 225)};
        cv::Point2f centerPt = cv::Point2f(img.cols / 2, img.rows);
        for ( std::size_t i = 0; i < contours_blue.size(); i++ )
        {        
          cv::Moments M = cv::moments(contours_blue[i]);
          float center_x = static_cast<float>(M.m10 / (M.m00 + 1e-5));
          float center_y = static_cast<float>(M.m01 / (M.m00 + 1e-5));

          // Approximate a poly shape
          cv::approxPolyDP(contours_blue[i],PapperContour,cv::arcLength(contours_blue[i],true)*0.02,true);

          // Draw the blue papper
          cv::polylines(img, PapperContour, true, cv::Scalar(0, 255, 0), 2);

          // Ignore too small contour
          if (cv::contourArea(PapperContour) <=2500 && (std::fabs(center_x) <= img.cols / 5.0f || std::fabs(center_x - img.cols) <= img.cols / 5.0f) ){
            cv::polylines(img, PapperContour, true, cv::Scalar(128, 0, 128), 2);
            continue;
          }

          // Ignore too large contour
          // if (cv::contourArea(PapperContour) >40000 ){
          //   cv::polylines(img, PapperContour, true, cv::Scalar(0, 0, 128), 2);
          //   continue;
          // }

          // Ignore when the contour stay really close to the bottom border
          if (std::fabs(center_y - img.rows) <= img.rows / 4.0f || std::fabs(center_y) <= img.rows / 4.0f ){
            cv::polylines(img, PapperContour, true, cv::Scalar(0, 255, 255), 2);
            continue;
          }

          if (cv::arcLength(PapperContour,true) >= 200 && cv::matchShapes(ContourCompare,PapperContour,cv::CONTOURS_MATCH_I1,0.0) >= 0.3){
            cv::polylines(img, PapperContour, true, cv::Scalar(0, 0, 255), 2);
            centerPt = cv::Point2f(center_x , center_y);
            break;
          }
        }

        // Show the result
        if (verbose){
          cv::imshow("Blue papper contour", img);
          // cv::imshow("Result", img);
          cv::waitKey(1);
        }

        // Send message to another microservice
        opendlv::logic::perception::DetectionProperty message;
        message.sampleId(0); //0 stands for the papper
        message.detectionId(1); //0 stands for yellow, 1 for blue
        message.property(std::to_string(centerPt.x) + ";" + std::to_string(centerPt.y)); //property stands for the position
        od4.send(message);
      }
    }
    retCode = 0;
  }
  return retCode;
}

