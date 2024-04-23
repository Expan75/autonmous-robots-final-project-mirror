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


bool comparePointx(cv::Point2f P1, cv::Point2f P2)
{
  return (P1.x < P2.x);
}

bool comparePointy(cv::Point2f P1, cv::Point2f P2)
{
  return (P1.y < P2.y);
}
bool comparePointInvy(cv::Point2f P1, cv::Point2f P2)
{
  return (P1.y > P2.y);
}
bool compareVectorPointx(std::vector<cv::Point> C1, std::vector<cv::Point> C2)
{  
  cv::Moments M1 = cv::moments(C1);
  float center_x1 = static_cast<float>(M1.m10 / (M1.m00 + 1e-5));
  cv::Moments M2 = cv::moments(C2);
  float center_x2 = static_cast<float>(M2.m10 / (M2.m00 + 1e-5));
  return (center_x1 < center_x2);
}

bool compareVectorPointy(std::vector<cv::Point> C1, std::vector<cv::Point> C2)
{
  cv::Moments M1 = cv::moments(C1);
  float center_y1 = static_cast<float>(M1.m01 / (M1.m00 + 1e-5));
  cv::Moments M2 = cv::moments(C2);
  float center_y2 = static_cast<float>(M2.m01 / (M2.m00 + 1e-5));
  return (center_y1 < center_y2);
}

bool compareVectorPointInvy(std::vector<cv::Point> C1, std::vector<cv::Point> C2)
{
  cv::Moments M1 = cv::moments(C1);
  float center_y1 = static_cast<float>(M1.m01 / (M1.m00 + 1e-5));
  cv::Moments M2 = cv::moments(C2);
  float center_y2 = static_cast<float>(M2.m01 / (M2.m00 + 1e-5));
  return (center_y1 > center_y2);
}

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

        // Turn the color to HSV
        cv::Mat hsv;
        cv::cvtColor(sharpenImage, hsv, cv::COLOR_BGR2HSV);

        // Only focus on the bottom 2/3 part of the image to avoid noise
        cv::Rect roi(0, hsv.rows / 3 , hsv.cols, hsv.rows * 2 / 3 );
        cv::Mat bottom_half = hsv(roi);

        // Extract the yellow and blue cones out of the image
        cv::Mat yC_sharpen;
        cv::Mat bC_sharpen;
        cv::Mat bC_sharpen_1;
        cv::Mat bC_sharpen_2;
        cv::inRange(bottom_half, cv::Scalar(20,50,50), cv::Scalar(50,255,255), yC_sharpen);  
        cv::inRange(bottom_half, cv::Scalar(110,50,50), cv::Scalar(130,255,255), bC_sharpen);  

        // Further process the result to make more precise result
        // Yellow cones part
        cv::Mat yC_Mat;
        cv::Mat element = getStructuringElement( cv::MORPH_RECT , cv::Size( 3,3 ), cv::Point( 0,0 ) );
        img(roi).copyTo(yC_Mat,yC_sharpen);
        cv::cvtColor(yC_Mat, yC_Mat, cv::COLOR_BGR2GRAY );
        cv::GaussianBlur( yC_Mat, yC_Mat, cv::Size(5,5), 0 );
        cv::threshold(yC_Mat, yC_Mat, 40, 255, cv::THRESH_TOZERO );
        cv::morphologyEx( yC_Mat, yC_Mat, cv::MORPH_CLOSE , element );
        cv::dilate(yC_Mat,yC_Mat,element, cv::Point(-1,-1),3,1,1);

        // Blue cones part
        cv::inRange(bottom_half, cv::Scalar(130,50,50), cv::Scalar(150,255,255), bC_sharpen_1);  
        cv::inRange(bottom_half, cv::Scalar(150,50,50), cv::Scalar(170,255,255), bC_sharpen_2);  
        cv::addWeighted( bC_sharpen_1, 0.25, bC_sharpen, 0.75, 0.0, bC_sharpen);
        cv::addWeighted( bC_sharpen_2, 0.1, bC_sharpen, 0.9, 0.0, bC_sharpen);
        cv::morphologyEx( bC_sharpen, bC_sharpen, cv::MORPH_CLOSE , element );
        cv::dilate(bC_sharpen,bC_sharpen,element, cv::Point(-1,-1),3,1,1);
        cv::threshold(bC_sharpen, bC_sharpen, 150, 255, cv::THRESH_TOZERO );
        
        // Find the contour of the extract cones image
        std::vector<std::vector<cv::Point>> contours_blue;
        std::vector<std::vector<cv::Point>> contours_yellow;
        cv::findContours(bC_sharpen, contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(yC_Mat, contours_yellow, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::sort(contours_blue.begin(), contours_blue.end(), compareVectorPointInvy);     
        std::sort(contours_yellow.begin(), contours_yellow.end(), compareVectorPointInvy);

        // Iterate over all yellow cones to process noises
        cv::Mat PrintResult = img;
        std::vector<cv::Point2f> usableYellowCones;   
        cv::Rect IndicateRect;
        for ( std::size_t i = 0; i < contours_yellow.size(); i++ )
        {
          cv::Moments M = cv::moments(contours_yellow[i]);
          float center_x = static_cast<float>(M.m10 / (M.m00 + 1e-5));
          float center_y = static_cast<float>(M.m01 / (M.m00 + 1e-5));

          // Ignore car 
          if ( bottom_half.rows - center_y <= bottom_half.rows / 4 && abs(center_x - bottom_half.cols / 2) <= bottom_half.rows / 4.8 ){
            continue;
          }

          // Ignore too close to border noise
          if ( center_y <= bottom_half.rows / 4 ){
            continue;
          }
          
          // Ignore drastically change slope     
          if ( usableYellowCones.size() != 0 && center_x - usableYellowCones[usableYellowCones.size()-1].x > bottom_half.cols / 2){
            continue;
          }
          
          // Inidicate right cones in rectangle shape and add it to array
          IndicateRect = cv::boundingRect(contours_yellow[i]);
          cv::rectangle(PrintResult, cv::Point(IndicateRect.x,IndicateRect.y + img.rows / 3), cv::Point(IndicateRect.x + IndicateRect.width,IndicateRect.y + IndicateRect.height + img.rows / 3), cv::Scalar(0,255,255), 5);
          usableYellowCones.push_back(cv::Point2f(center_x,center_y));
        }  

        // Further Ignore wrong position noise
        for ( std::size_t i = 0; i < usableYellowCones.size(); i++ ) {
          if ( usableYellowCones[0].x > bottom_half.cols / 2 ){
            usableYellowCones.clear();
            break;
          }  
        }

        // Iterate over all blue cones to process noises 
        std::vector<cv::Point2f> usableBlueCones;   
        for ( std::size_t i = 0; i < contours_blue.size(); i++ )
        {        
          cv::Moments M = cv::moments(contours_blue[i]);
          float center_x = static_cast<float>(M.m10 / (M.m00 + 1e-5));
          float center_y = static_cast<float>(M.m01 / (M.m00 + 1e-5));

          // Ignore too close to border noise for the bottom part (Including the car)
          if ( bottom_half.rows - center_y <= bottom_half.rows / 4 && ( cv::contourArea(contours_blue[i]) <= bottom_half.rows * bottom_half.cols / 600 || abs(center_x - bottom_half.cols / 2) <= bottom_half.rows / 4 || center_x - bottom_half.cols / 2 <= 0 ) ){
            continue;
          }

          // Ignore too close to border noise
          if ( center_y <= bottom_half.rows / 4 ){
            continue;
          }

          // Ignore wrong position noise
          if ( usableYellowCones.size() !=0 &&  center_x  <= usableYellowCones[usableYellowCones.size() / 2].x ){
            continue;
          }

          // Ignore drastically change point  
          if ( usableBlueCones.size() != 0 && center_x - usableBlueCones[usableBlueCones.size()-1].x > bottom_half.cols* 5 / 12){
            continue;
          }

          // Put the point into the vector 
          IndicateRect = cv::boundingRect(contours_blue[i]);
          cv::rectangle(PrintResult, cv::Point(IndicateRect.x,IndicateRect.y + img.rows / 3), cv::Point(IndicateRect.x + IndicateRect.width,IndicateRect.y + IndicateRect.height + img.rows / 3), cv::Scalar(255,0,0), 5);
          usableBlueCones.push_back(cv::Point2f(center_x,center_y));
        }

        // Show the result
        if (verbose){
          cv::imshow("Result", PrintResult);
          cv::waitKey(1);
        }

        // Send message to another microservice
        opendlv::logic::perception::DetectionProperty message;
        for ( std::size_t i = 0; i < usableBlueCones.size(); i++ )
        {
          message.sampleId(i); //0 stands for the first point
          message.detectionId(1); //0 stands for yellow cone, 1 for the blue cone
          message.property(std::to_string(usableBlueCones[i].x) + ";" + std::to_string(usableBlueCones[i].y)); //property stands for the position
          od4.send(message);
        }
        for ( std::size_t i = 0; i < usableYellowCones.size(); i++ )
        {
          message.sampleId(i); //0 stands for the first point
          message.detectionId(0); //0 stands for yellow cone, 1 for the blue cone
          message.property(std::to_string(usableYellowCones[i].x) + ";" + std::to_string(usableYellowCones[i].y)); //property stands for the position
          od4.send(message);
        }

        /////////////////Steering Algorithm Part/////////////////////////
        // Follow part might be dealed with another microservice later
        // Find the first and the last point and find the final aim point
        cv::Point2f yellowConeFpt, yellowConeLpt, blueConeFpt, blueConeLpt;
        cv::Point2f tempFpt, tempLpt, AimPt, CenterPt;
        if ( usableYellowCones.size() > 0 && usableBlueCones.size() > 0 )
        {
          yellowConeFpt = cv::Point2f(usableYellowCones[0].x, usableYellowCones[0].y + hsv.rows / 2);
          yellowConeLpt = cv::Point2f(usableYellowCones[usableYellowCones.size()-1].x, usableYellowCones[usableYellowCones.size()-1].y + hsv.rows / 2);
          blueConeFpt = cv::Point2f(usableBlueCones[0].x, usableBlueCones[0].y + hsv.rows / 2);
          blueConeLpt = cv::Point2f(usableBlueCones[usableBlueCones.size()-1].x, usableBlueCones[usableBlueCones.size()-1].y + hsv.rows / 2);
        }
        else if ( usableYellowCones.size() > 0 && usableBlueCones.size() == 0 )
        {
          yellowConeFpt = cv::Point2f(usableYellowCones[0].x, usableYellowCones[0].y + hsv.rows / 2);
          yellowConeLpt = cv::Point2f(usableYellowCones[usableYellowCones.size()-1].x, usableYellowCones[usableYellowCones.size()-1].y + hsv.rows / 2);
          blueConeFpt = cv::Point2f(hsv.cols, hsv.rows);
          blueConeLpt = cv::Point2f(hsv.cols / 2, hsv.rows);
        }
        else if( usableYellowCones.size() == 0 && usableBlueCones.size() > 0 )
        {  
          yellowConeFpt = cv::Point2f(0, hsv.rows);
          yellowConeLpt = cv::Point2f(hsv.cols / 2, hsv.rows);
          blueConeFpt = cv::Point2f(usableBlueCones[0].x, usableBlueCones[0].y + hsv.rows / 2);
          blueConeLpt = cv::Point2f(usableBlueCones[usableBlueCones.size()-1].x, usableBlueCones[usableBlueCones.size()-1].y + hsv.rows / 2);   
        }
        tempFpt = cv::Point2f((yellowConeFpt.x + blueConeFpt.x) / 2, (yellowConeFpt.y + blueConeFpt.y) / 2);
        tempLpt = cv::Point2f((yellowConeLpt.x + blueConeLpt.x) / 2, (yellowConeLpt.y + blueConeLpt.y) / 2);
        if ( usableYellowCones.size() == 0 )
        {
          tempFpt = cv::Point2f((yellowConeFpt.x + tempFpt.x) / 2, (yellowConeFpt.y + tempFpt.y) / 2);
          tempLpt = cv::Point2f((yellowConeLpt.x + tempLpt.x) / 2, (yellowConeLpt.y + tempLpt.y) / 2);
        }
        AimPt = cv::Point2f((tempFpt.x + tempLpt.x) / 2, (tempFpt.y + tempLpt.y) / 2);
        CenterPt = cv::Point2f(hsv.cols / 2, hsv.rows);        

        // Display image.
        if (verbose) {
          cv::Mat ResultMat = img.clone();
          // Temp: line between cone    
          // cv::line(ResultMat, yellowConeFpt, yellowConeLpt, cv::Scalar(0, 255, 255), 2, cv::LINE_8); 
          // cv::line(ResultMat, blueConeFpt, blueConeLpt, cv::Scalar(255, 0, 0), 2, cv::LINE_8); 
          // cv::line(ResultMat, yellowConeFpt, blueConeFpt, cv::Scalar(0, 0, 255), 2, cv::LINE_8); 
          // cv::line(ResultMat, yellowConeLpt, blueConeLpt, cv::Scalar(0, 0, 255), 2, cv::LINE_8); 

          cv::line(ResultMat, CenterPt, cv::Point2f(hsv.cols / 2, 0), cv::Scalar(0, 0, 255), 2, cv::LINE_8); 
          cv::line(ResultMat, CenterPt, AimPt, cv::Scalar(0, 255, 0), 2, cv::LINE_8); 
          // cv::imshow("Process Result", ResultMat);
          // cv::imshow(sharedMemory->name().c_str(), ResultMat);
          // cv::waitKey(1);
        }

        ////////////////////////////////////////////////////////////////
        // Do something with the distance readings if wanted.
        {
          std::lock_guard<std::mutex> lck(distancesMutex);
          // std::cout << "front = " << front << ", "
          //           << "rear = " << rear << ", "
          //           << "left = " << left << ", "
          //           << "right = " << right << "." << std::endl;
        }

        ////////////////////////////////////////////////////////////////
        // Example for creating and sending a message to other microservices;
        // can be removed when not needed.
        opendlv::proxy::AngleReading ar;
        ar.angle(123.45f);
        od4.send(ar);

        ////////////////////////////////////////////////////////////////
        // Steering and acceleration/decelration.
        //
        // Uncomment the following lines to steer; range: +38deg (left) ..
        // -38deg (right). Value groundSteeringRequest.groundSteering must be
        // given in radians (DEG/180. * PI).

        // Calculate angle from detected cones
        float Angle;
        Angle = std::atan2(CenterPt.x - AimPt.x, CenterPt.y - AimPt.y);
        // Angle = Angle * 38 / 180; // Normalize to 38 degree
        Angle = Angle * 38 / 90; // Normalize to 38 degree
        opendlv::proxy::GroundSteeringRequest gsr;
        gsr.groundSteering(Angle);
        od4.send(gsr);

        // Uncomment the following lines to accelerate/decelerate; range: +0.25
        // (forward) .. -1.0 (backwards). Be careful!
        opendlv::proxy::PedalPositionRequest ppr;
        ppr.position(0);
        od4.send(ppr);  
      }
    }
    retCode = 0;
  }
  return retCode;
}

