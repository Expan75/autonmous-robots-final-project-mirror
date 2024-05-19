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
    uint16_t const CID = std::stoi(cmd.at("cid"));
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
      cluon::OD4Session od4{CID};
      
      float steer_for_show{0.0f};
      auto onGroundSteeringRequest{[&steer_for_show](
          cluon::data::Envelope &&envelope)
        {
          auto groundSteeringAngleRequest = 
            cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(
                std::move(envelope));
          steer_for_show = groundSteeringAngleRequest.groundSteering();
        }};

      float pedal_for_show{0.0f};
      auto onPedalPositionRequest{[&pedal_for_show](
          cluon::data::Envelope &&envelope)
        {
          auto pedalPositionRequest = 
            cluon::extractMessage<opendlv::proxy::PedalPositionRequest>(
                std::move(envelope));
          pedal_for_show = pedalPositionRequest.position();
        }};

      od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(),
          onGroundSteeringRequest);
      od4.dataTrigger(opendlv::proxy::PedalPositionRequest::ID(),
          onPedalPositionRequest);


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
        // cv::imshow("Blue papper contour", img); 

        // Turn the color to HSV
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        // Specify rectangle area
        cv::Rect roi(0,hsv.rows / 2,hsv.cols,hsv.rows / 2);

        // Create krenel for morphology process
        cv::Mat element = getStructuringElement( cv::MORPH_RECT , cv::Size( 3,3 ), cv::Point( 0,0 ) );

        // Extract the blue papper
        cv::Mat bC_papper;
        cv::inRange(hsv(roi), cv::Scalar(100,37,77), cv::Scalar(130,150,150), bC_papper); // blue papper
        cv::morphologyEx( bC_papper, bC_papper, cv::MORPH_CLOSE , element );
        cv::dilate(bC_papper,bC_papper,element, cv::Point(-1,-1),3,1,1);
        
        // Find the contour of the extract papper image
        std::vector<std::vector<cv::Point>> contours_blue;
        cv::findContours(bC_papper, contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Extract the green papper
        cv::Mat gC_papper;
        cv::inRange(hsv(roi), cv::Scalar(25,88,115), cv::Scalar(35,204,230), gC_papper);  // green papper
        cv::morphologyEx( gC_papper, gC_papper, cv::MORPH_CLOSE , element );
        cv::dilate(gC_papper,gC_papper,element, cv::Point(-1,-1),3,1,1);
        
        // Find the contour of the extract papper image
        std::vector<std::vector<cv::Point>> contours_green;
        cv::findContours(gC_papper, contours_green, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Iterate over all contours to find the blue papper
        std::vector<cv::Point> PapperContour;   
        cv::Point2f centerPt_blue = cv::Point2f(0.0f, 0.0f);
        int bsizeCount = 0;
        for ( std::size_t i = 0; i < contours_blue.size(); i++ )
        {        
          // Approximate a poly shape
          cv::approxPolyDP(contours_blue[i],PapperContour,cv::arcLength(contours_blue[i],true)*0.02,true);
          
          std::vector<cv::Point> Hull;
          cv::convexHull(PapperContour,Hull);
          cv::Moments M = cv::moments(Hull);
          float center_x = static_cast<float>(M.m10 / (M.m00 + 1e-5));
          float center_y = static_cast<float>(M.m01 / (M.m00 + 1e-5));

          if ( cv::arcLength(Hull,true) < 100 && static_cast<float>(roi.height) - center_y <= 150.0f ){
            cv::polylines(img(roi), Hull, true, cv::Scalar(0,0,255));
            continue;
          }

          // Ignore car 
          if ( static_cast<float>(roi.height) - center_y <= 127.0f && static_cast<float>(roi.width) / 2.0f - center_x <= 170.0f && static_cast<float>(roi.width) / 2.0f - center_x > 0.0f ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(0,255,0));
            continue;
          }
          
          if ( static_cast<float>(roi.height) - center_y <= 25.0f ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(255,255,0));
            continue;
          }

          // Draw the blue papper
          cv::putText(img(roi), std::to_string(center_x) + ", "+ std::to_string(center_y),cv::Point2f(center_x,center_y),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,255));
          cv::polylines(img(roi), Hull, true, cv::Scalar(255, 0, 0), 2);

          // Record center point
          centerPt_blue.x += center_x;
          centerPt_blue.y += center_y;
          bsizeCount++;
        }

        if (bsizeCount == 0){
          centerPt_blue = cv::Point2f(hsv.cols / 2, hsv.rows); // Reset center point
        }
        else{
          centerPt_blue.x = centerPt_blue.x / bsizeCount;
          centerPt_blue.y = centerPt_blue.y / bsizeCount + hsv.rows / 2;
          cv::line(img,cv::Point2f(img.cols / 2,img.rows),centerPt_blue,cv::Scalar(255,0,0));
        }

        cv::Point2f centerPt_green = cv::Point2f(0.0f, 0.0f);
        int gsizeCount = 0;
        for ( std::size_t i = 0; i < contours_green.size(); i++ )
        {        
          // Approximate a poly shape
          cv::approxPolyDP(contours_green[i],PapperContour,cv::arcLength(contours_green[i],true)*0.02,true);
          
          std::vector<cv::Point> Hull;
          cv::convexHull(PapperContour,Hull);
          cv::Moments M = cv::moments(Hull);
          float center_x = static_cast<float>(M.m10 / (M.m00 + 1e-5));
          float center_y = static_cast<float>(M.m01 / (M.m00 + 1e-5));

          if ( cv::arcLength(Hull,true) < 50 && static_cast<float>(roi.height) - center_y <= 150.0f ){
            cv::polylines(img(roi), Hull, true, cv::Scalar(0,0,255));
            continue;
          }

          // Ignore car 
          if ( static_cast<float>(roi.height) - center_y <= 127.0f && static_cast<float>(roi.width) / 2.0f - center_x <= 170.0f && static_cast<float>(roi.width) / 2.0f - center_x > 0.0f ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(0,255,0));
            continue;
          }
          
          if ( static_cast<float>(roi.height) - center_y <= 25.0f ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(255,255,0));
            continue;
          }

          // Draw the green papper
          cv::putText(img(roi), std::to_string(center_x) + ", "+ std::to_string(center_y),cv::Point2f(center_x,center_y),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,255));
          cv::polylines(img(roi), Hull, true, cv::Scalar(0, 255, 0), 2);

          // Record center point
          centerPt_green.x += center_x;
          centerPt_green.y += center_y;
          gsizeCount++;
        }

        if (gsizeCount == 0){
          centerPt_green = cv::Point2f(hsv.cols / 2, hsv.rows); // Reset center point
        }
        else{
          centerPt_green.x = centerPt_green.x / gsizeCount;
          centerPt_green.y = centerPt_green.y / gsizeCount + hsv.rows / 2;
          cv::line(img,cv::Point2f(img.cols / 2,img.rows),centerPt_green,cv::Scalar(0,255,0));
        }

        // Show the result
        if (verbose){
          cv::putText(img, "pedal: "+ std::to_string(pedal_for_show),cv::Point2f(img.cols / 2,img.rows - 40),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,255));
          cv::putText(img, "steering: "+ std::to_string(steer_for_show),cv::Point2f(img.cols / 2,img.rows - 20),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,255));
          cv::imshow("Blue/Green papper contour", img);
          // cv::imshow("Result", img);
          cv::waitKey(1);
        }

        // Send message to another microservice
        opendlv::logic::perception::DetectionProperty message;
        float dist = (gsizeCount != 0) ? 
        std::sqrt(std::pow(img.cols / 2 - centerPt_green.x,2.0f) + std::pow(img.rows - centerPt_green.y,2.0f))
        : -1.0f;
        float aimDirection = std::atan2(img.cols / 2 - centerPt_green.x,img.rows - centerPt_green.y); //Direction in rad
        message.sampleId(0); //0 stands for the papper
        message.detectionId(0); //0 stands for green, 1 for blue
        message.property(std::to_string(dist) + ";" + std::to_string(aimDirection)); //property stands for dist / angle
        od4.send(message);
        dist = (bsizeCount != 0) ? 
        std::sqrt(std::pow(img.cols / 2 - centerPt_blue.x,2.0f) + std::pow(img.rows - centerPt_blue.y,2.0f))
        : -1.0f;
        aimDirection = std::atan2(img.cols / 2 - centerPt_blue.x,img.rows - centerPt_blue.y); //Direction in rad
        message.sampleId(0); //0 stands for the papper
        message.detectionId(1); //0 stands for green, 1 for blue
        message.property(std::to_string(dist) + ";" + std::to_string(aimDirection)); //property stands for dist / angle
        od4.send(message);
      }
    }
    retCode = 0;
  }
  return retCode;
}

