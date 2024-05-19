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
        cv::Point2f RefPt = cv::Point2f(1280 / 2, 720);    

        // Calculate the acc need to implement and send it range: +0.25 (forward) .. -1.0 (backwards)
        double dist = std::sqrt(std::pow(RefPt.x - center_x, 2) + std::pow(RefPt.y - center_y, 2));
        // double farestDist = std::sqrt(std::pow(1280 / 2, 2) + std::pow(720, 2));
        double distDeviation = (dist - 420) / 420 * 0.45 + 0.45;
        opendlv::proxy::PedalPositionRequest ppr;
        ppr.position(static_cast<float>(distDeviation));
        od4.send(ppr);      
        
        // Calculate the angle need to turn and send it range: + 38 to - 38 degree
        // float AngleDisplay;
        float Angle = std::atan2(RefPt.x - center_x, RefPt.y - center_y);
        // AngleDisplay = Angle;
        Angle = Angle * 30 / 90; // Normalize to 30 degree to avoid saturation
        opendlv::proxy::GroundSteeringRequest gsr;
        gsr.groundSteering(Angle);
        od4.send(gsr);

        // std::cout << "Got object position x:" << RefPt.x - center_x << " ,y: " << RefPt.y - center_y << std::endl;
        std::cout << "Angle Turn:" << Angle / 3.14 * 180 << " , Acc: " << distDeviation << std::endl;
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
        cv::imshow("detected circles", img);

        // Show the result
        if (verbose){
          // cv::imshow("Result", img);
          cv::waitKey(1);
        }

        // Send message to another microservice
        opendlv::logic::perception::DetectionProperty message;
        message.sampleId(0); //0 stands for the papper
        message.detectionId(1); //0 stands for yellow, 1 for blue
        message.property(std::to_string(AimPt.x) + ";" + std::to_string(AimPt.y)); //property stands for the position
        od4.send(message);
      }
    }
    retCode = 0;
  }
  return retCode;
}

