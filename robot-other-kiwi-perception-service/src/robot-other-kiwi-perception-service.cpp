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

        // Turn the color to gray scale and do blurring
        cv::Mat bnImage;
        cv::bitwise_not(img,bnImage);
        cv::Mat grayImage;
        cv::cvtColor(bnImage, grayImage, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(grayImage,grayImage,cv::Size(3,3),0);
        // cv::imshow("Canny:", grayImage);

        // cv::Mat Canny_Image;
        // cv::Canny(bnImage,Canny_Image,30,30*3,3);
        // // cv::GaussianBlur(bnImage,bnImage,cv::Size(3,3),0);
        // cv::imshow("Canny:", Canny_Image);
        
        std::vector<cv::Vec3f> circles;
        HoughCircles(grayImage, circles, cv::HOUGH_GRADIENT, 1,
        grayImage.rows/64, // change this value to detect circles with different distances to each other
        100, 30, 
        0, 30 // change the last two parameters
        // (min_radius & max_radius) to detect larger circles
        );

        // Draw circle on the image
        cv::Point2f AimPt;
        int nCounter{0};
        for( size_t i = 0; i < circles.size(); i++ )
        {
          cv::Vec3f c = circles[i];

          // circle center
          cv::Point2f center = cv::Point2f(c[0], c[1]);
          if ( img.rows - c[1] <= img.rows / 6 || c[1] <= img.rows / 2 ){
            continue;
          }
          circle( img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
          AimPt = cv::Point2f(AimPt.x + c[0],AimPt.y + c[1]);
          nCounter++;

          // circle outline
          int radius = static_cast<int>(c[2]);
          circle( img, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
        }
        AimPt = (nCounter == 0) ? cv::Point2f(img.cols / 2, img.rows) : cv::Point2f(AimPt.x / nCounter, AimPt.y / nCounter);

        cv::line(img,cv::Point2f(img.cols / 2,img.rows),AimPt,cv::Scalar(0,255,0));

        // Show the result
        if (verbose){
          cv::putText(img, "pedal: "+ std::to_string(pedal_for_show),cv::Point2f(img.cols / 2,img.rows - 40),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,255));
          cv::putText(img, "steering: "+ std::to_string(steer_for_show),cv::Point2f(img.cols / 2,img.rows - 20),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,255));
          cv::imshow("detected circles", img);
          // cv::imshow("Result", img);
          cv::waitKey(1);
        }

        // Send message to another microservice
        opendlv::logic::perception::DetectionProperty message;
        float dist = (nCounter != 0) ? 
        std::sqrt(std::pow(img.cols / 2 - AimPt.x,2.0f) + std::pow(img.rows - AimPt.y,2.0f)): -1.0f;
        float aimDirection = std::atan2(img.cols / 2 - AimPt.x,img.rows - AimPt.y); //Direction in rad
        message.sampleId(0); //0 stands for the papper
        message.detectionId(0); //0 stands for circles
        message.property(std::to_string(dist) + ";" + std::to_string(aimDirection)); //property stands for the dist and aimdirection
        od4.send(message);
      }
    }
    retCode = 0;
  }
  return retCode;
}

