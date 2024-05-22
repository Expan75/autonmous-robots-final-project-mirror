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


bool comparePointx(cv::Point P1, cv::Point P2)
{
  return (P1.x < P2.x);
}

bool comparePointy(cv::Point P1, cv::Point P2)
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

    int hBlueLow = (cmd.count("hBlueLow") != 0) ? std::stoi(cmd["hBlueLow"]) : 80;
    int sBlueLow = (cmd.count("sBlueLow") != 0) ? std::stoi(cmd["sBlueLow"]) : 0;
    int vBlueLow = (cmd.count("vBlueLow") != 0) ? std::stoi(cmd["vBlueLow"]) : 0;
    int hYellowLow = (cmd.count("hYellowLow") != 0) ? std::stoi(cmd["hYellowLow"]) : 20;
    int sYellowLow = (cmd.count("sYellowLow") != 0) ? std::stoi(cmd["sYellowLow"]) : 195;
    int vYellowLow = (cmd.count("vYellowLow") != 0) ? std::stoi(cmd["vYellowLow"]) : 110;
    
    int hBlueHigh = (cmd.count("hBlueHigh") != 0) ? std::stoi(cmd["hBlueHigh"]) : 180;
    int sBlueHigh = (cmd.count("sBlueHigh") != 0) ? std::stoi(cmd["sBlueHigh"]) : 200;
    int vBlueHigh = (cmd.count("vBlueHigh") != 0) ? std::stoi(cmd["vBlueHigh"]) : 60;
    int hYellowHigh = (cmd.count("hYellowHigh") != 0) ? std::stoi(cmd["hYellowHigh"]) : 30;
    int sYellowHigh = (cmd.count("sYellowHigh") != 0) ? std::stoi(cmd["sYellowHigh"]) : 255;
    int vYellowHigh = (cmd.count("vYellowHigh") != 0) ? std::stoi(cmd["vYellowHigh"]) : 255;

    float YBRatio = (cmd.count("YBRatio") != 0) ? std::stof(cmd["YBRatio"]) : 0.6f;

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
        cv::Mat testMat; // For DODO test

        // Wait for a notification of a new frame.
        sharedMemory->wait();

        // Lock the shared memory.
        sharedMemory->lock();
        {
          // cv::Mat wrapped(height, width, CV_8UC3, sharedMemory->data());
          cv::Mat wrapped_1(height, width, CV_8UC4, sharedMemory->data());
          // img = wrapped.clone();
          testMat = wrapped_1.clone();
        }
        sharedMemory->unlock();
        cv::cvtColor(testMat, img, cv::COLOR_BGRA2BGR);
        // cv::imshow("Test Image: ", testMat);

        // Turn the color to HSV
        // cv::bitwise_not(img,img);
        // cv::imwrite("/home/opendlv/data/Project/team11-project-monorepo/robot-perception-service/ScreenShot.png",img);
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        // cv::Rect roi(0, hsv.rows / 2 , hsv.cols, hsv.rows / 2 );
        cv::Rect roi(0, hsv.rows * 7 / 12 , hsv.cols, hsv.rows * 5 / 12 ); // For DODO Test
        // cv::Rect roi_center(hsv.cols / 4, hsv.rows * 5 / 9, hsv.cols / 2, hsv.rows /9);
        // hsv = hsv(roi);

        // Extract the yellow and blue cones out of the image
        cv::Mat yC_sharpen;
        cv::Mat bC_sharpen;
        cv::inRange(hsv(roi), cv::Scalar(hYellowLow,sYellowLow,vYellowLow), cv::Scalar(hYellowHigh,sYellowHigh,vYellowHigh), yC_sharpen);    // For yellow cones
        cv::inRange(hsv(roi), cv::Scalar(hBlueLow,sBlueLow,vBlueLow), cv::Scalar(hBlueHigh,sBlueHigh,vBlueHigh), bC_sharpen);  // For blue cones
        cv::inRange(hsv(roi), cv::Scalar(25,245,245), cv::Scalar(35,255,255), yC_sharpen);    // For yellow cones of DODO test SIL
        cv::inRange(hsv(roi), cv::Scalar(115,245,245), cv::Scalar(125,255,255), bC_sharpen);  // For blue cones of DODO test SIL
        cv::Mat element = getStructuringElement( cv::MORPH_RECT , cv::Size( 3,3 ), cv::Point( 0,0 ) );
        cv::morphologyEx( yC_sharpen, yC_sharpen, cv::MORPH_CLOSE , element );
        cv::morphologyEx( yC_sharpen, yC_sharpen, cv::MORPH_OPEN , element );
        cv::dilate(yC_sharpen,yC_sharpen,element, cv::Point(-1,-1),3,1,1);
        cv::morphologyEx( bC_sharpen, bC_sharpen, cv::MORPH_CLOSE , element );
        cv::morphologyEx( bC_sharpen, bC_sharpen, cv::MORPH_OPEN , element );
        cv::dilate(bC_sharpen,bC_sharpen,element, cv::Point(-1,-1),3,1,1);
        // cv::imshow("Cone: ", bC_sharpen);
        // cv::imshow("Cone: ", bC_sharpen);

        cv::Mat Cone_Canny_yellow;
        cv::Mat Cone_Canny_blue;
        Canny( yC_sharpen, Cone_Canny_yellow, 50, 50*3, 3 );
        Canny( bC_sharpen, Cone_Canny_blue, 50, 50*3, 3 );
        // Iterate over all yellow cones to process noises
        std::vector<std::vector<cv::Point>> contours_blue;
        std::vector<cv::Point2f> usableBlueCones;  
        cv::findContours(Cone_Canny_blue, contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for ( std::size_t i = 0; i < contours_blue.size(); i++ )
        {                 
          std::vector<cv::Point> Poly;
          cv::approxPolyDP(contours_blue[i],Poly,cv::arcLength(contours_blue[i],true)*0.04,true);

          std::vector<cv::Point> Hull;
          cv::convexHull(Poly,Hull);

          if ( Hull.size() > 10 ||  Hull.size() < 3 ){
            continue;
          }

          if ( cv::arcLength(Hull,true) > 500 || cv::arcLength(Hull,true) < 20 ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(0,0,255));
            continue;
          }

          // Ignore car 
          cv::Moments M = cv::moments(Hull);
          float center_x = static_cast<float>(M.m10 / (M.m00 + 1e-5));
          float center_y = static_cast<float>(M.m01 / (M.m00 + 1e-5));
          if ( static_cast<float>(roi.height) - center_y <= 127.0f && std::fabs(static_cast<float>(roi.width) / 2.0f - center_x) <= 160.0f ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(0,255,0));
            continue;
          }
          
          if ( static_cast<float>(roi.height) - center_y <= 15.0f ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(255,255,0));
            continue;
          }
          
          cv::Rect IndicateRect = cv::boundingRect(Hull);
          // cv::putText(img(roi), std::to_string(center_x) + ", "+ std::to_string(center_y),cv::Point(IndicateRect.x,IndicateRect.y),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,0,0));
          cv::rectangle(img(roi), cv::Point(IndicateRect.x,IndicateRect.y), cv::Point(IndicateRect.x + IndicateRect.width,IndicateRect.y + IndicateRect.height), cv::Scalar(255,0,0), 5);
          usableBlueCones.push_back(cv::Point2f(center_x,center_y + roi.height)); 
          // cv::polylines(img(roi), Hull, true, cv::Scalar(0,255,255));
        }  
        std::sort(usableBlueCones.begin(),usableBlueCones.end(),comparePointx);

        // Iterate over all yellow cones to process noises
        std::vector<std::vector<cv::Point>> contours_yellow;
        std::vector<cv::Point2f> usableYellowCones;  
        cv::findContours(Cone_Canny_yellow, contours_yellow, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for ( std::size_t i = 0; i < contours_yellow.size(); i++ )
        {                 
          std::vector<cv::Point> Poly;
          cv::approxPolyDP(contours_yellow[i],Poly,cv::arcLength(contours_yellow[i],true)*0.04,true);

          std::vector<cv::Point> Hull;
          cv::convexHull(Poly,Hull);

          if ( Hull.size() > 10 ||  Hull.size() < 3 ){
            continue;
          }

          if ( cv::arcLength(Hull,true) > 500 || cv::arcLength(Hull,true) < 20 ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(0,0,255));
            continue;
          }

          // Ignore car 
          cv::Moments M = cv::moments(Hull);
          float center_x = static_cast<float>(M.m10 / (M.m00 + 1e-5));
          float center_y = static_cast<float>(M.m01 / (M.m00 + 1e-5));
          if ( static_cast<float>(roi.height) - center_y <= 127.0f && std::fabs(static_cast<float>(roi.width) / 2.0f - center_x) <= 160.0f ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(0,255,0));
            continue;
          }
          
          if ( static_cast<float>(roi.height) - center_y <= 15.0f ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(255,255,0));
            continue;
          }

          // if ( usableBlueCones.size() > 0 && center_x > usableBlueCones[usableBlueCones.size()-1].x ){
          if ( usableBlueCones.size() > 0 && center_x > static_cast<float>(roi.width) / 2.0f && static_cast<float>(roi.height) - center_y >= 190.0f ){
            // cv::polylines(img(roi), Hull, true, cv::Scalar(255,0,255));
            continue;
          }
          
          cv::Rect IndicateRect = cv::boundingRect(Hull);
          // cv::putText(img(roi), std::to_string(center_x) + ", "+ std::to_string(center_y),cv::Point(IndicateRect.x,IndicateRect.y),cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,255,255));
          cv::rectangle(img(roi), cv::Point(IndicateRect.x,IndicateRect.y), cv::Point(IndicateRect.x + IndicateRect.width,IndicateRect.y + IndicateRect.height), cv::Scalar(0,255,255), 5);
          usableYellowCones.push_back(cv::Point2f(center_x,center_y + roi.height)); 
          // cv::polylines(img(roi), Hull, true, cv::Scalar(0,255,255));
        }  

        // Sort the cone by y coordinate value before sended
        std::sort(usableBlueCones.begin(),usableBlueCones.end(),comparePointInvy);
        std::sort(usableYellowCones.begin(),usableYellowCones.end(),comparePointInvy);
        
        // Calculate Y deviation
        float yellowCone_baseX = (usableYellowCones.size() > 0) ? usableYellowCones[ 0 ].x : img.cols * 0.08f;
        float blueCone_baseX = (usableBlueCones.size() > 0) ? usableBlueCones[ 0 ].x : img.cols - img.cols * 0.08f;
        float reference_devY = img.cols / 2 - ( yellowCone_baseX + blueCone_baseX ) / 2.0f;

        // Calculate Aim direction        
        cv::Point2f middlePt_yellowCone = cv::Point2f(img.cols * 0.08f,img.rows - img.rows * 0.14f);
        for ( cv::Point2f Pt : usableYellowCones ){
          middlePt_yellowCone.x += Pt.x;
          middlePt_yellowCone.y += Pt.y;
        }
        middlePt_yellowCone.x = middlePt_yellowCone.x / (usableYellowCones.size() + 1);
        middlePt_yellowCone.y = middlePt_yellowCone.y / (usableYellowCones.size() + 1);

        cv::Point2f middlePt_blueCone = cv::Point2f(img.cols - img.cols * 0.08f,img.rows - img.rows * 0.14f);
        for ( cv::Point2f Pt : usableBlueCones ){
          middlePt_blueCone.x += Pt.x;
          middlePt_blueCone.y += Pt.y;
        }
        middlePt_blueCone.x = middlePt_blueCone.x / (usableBlueCones.size() + 1);
        middlePt_blueCone.y = middlePt_blueCone.y / (usableBlueCones.size() + 1);

         cv::Point2f AimPt = cv::Point2f(YBRatio * middlePt_yellowCone.x + (1.0f - YBRatio) * middlePt_blueCone.x
        , YBRatio * middlePt_yellowCone.y + (1.0f - YBRatio) * middlePt_blueCone.y);
        float aimDirection = std::atan2(img.cols / 2 - AimPt.x,img.rows - AimPt.y); //Direction in rad

        if (verbose){
          cv::line(img,cv::Point2f(img.cols / 2,img.rows),AimPt,cv::Scalar(0,255,0));
          cv::line(img,cv::Point2f(img.cols / 2,img.rows - 10),cv::Point2f(AimPt.x,img.rows - 10),cv::Scalar(0,0,255));
          cv::putText(img, "Pedal: " + std::to_string(pedal_for_show)
          , cv::Point(img.cols / 2,img.rows - 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0));
          cv::putText(img, "Steering: " + std::to_string(steer_for_show)
          , cv::Point(img.cols / 2,img.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0));  

          cv::imshow("Image: ", img);
          // cv::imshow("Cone: ", img);
          // cv::imshow("Yellow Cone being thresholded: ", th_img);
          cv::waitKey(5);
        }        
        
        // Send message to another microservice
        opendlv::logic::action::PreviewPoint pPtmessage;
        pPtmessage.distance(reference_devY);
        od4.send(pPtmessage);

        opendlv::logic::action::AimDirection aDirmessage;
        aDirmessage.azimuthAngle(aimDirection);
        od4.send(aDirmessage);
      }
    }
    retCode = 0;
  }
  return retCode;
}

