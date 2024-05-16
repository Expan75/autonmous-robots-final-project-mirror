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

      float yaw_mag{0.0f};
      auto onMagneticFieldReading{[&yaw_mag](
          cluon::data::Envelope &&envelope)
        {
          auto magReading = 
            cluon::extractMessage<opendlv::proxy::MagneticFieldReading>(
                std::move(envelope));
          float mag_x = magReading.magneticFieldX();
          float mag_y = magReading.magneticFieldY();
          yaw_mag = std::atan2(mag_y,mag_x);
        }};
      od4.dataTrigger(opendlv::proxy::MagneticFieldReading::ID(),
          onMagneticFieldReading);
      
      float yaw_gyro{0.0f};
      auto onAngularVelocityReading{[&yaw_gyro](
          cluon::data::Envelope &&envelope)
        {
          auto angVelReading = 
            cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(
                std::move(envelope));
          float deltaT = 0.01f;
          yaw_gyro += angVelReading.angularVelocityZ()*deltaT;
        }};
      od4.dataTrigger(opendlv::proxy::AngularVelocityReading::ID(),
        onAngularVelocityReading);
        
      float vX_acc{0.0f};
      float vY_acc{0.0f};
      float Y_acc{0.0f};
      auto onAccelerationReading{[&vX_acc, &vY_acc, &Y_acc](
          cluon::data::Envelope &&envelope)
        {
          auto accReading = 
            cluon::extractMessage<opendlv::proxy::AccelerationReading>(
                std::move(envelope));
          float deltaT = 0.01f;
          vX_acc += accReading.accelerationX()*deltaT;
          vY_acc += accReading.accelerationY()*deltaT;
          Y_acc += vY_acc*deltaT;
        }};
      od4.dataTrigger(opendlv::proxy::AccelerationReading::ID(),
        onAccelerationReading);

      double GPS_Lat{0};
      double GPS_Long{0};
      auto onGeodeticWgs84PositionReading{[&GPS_Lat, &GPS_Long](
          cluon::data::Envelope &&envelope)
        {
          auto GPSPostionReading = 
            cluon::extractMessage<opendlv::proxy::GeodeticWgs84PositionReading>(
                std::move(envelope));
          GPS_Lat = GPSPostionReading.latitude();
          GPS_Long = GPSPostionReading.longitude();
        }};
      od4.dataTrigger(opendlv::proxy::GeodeticWgs84PositionReading::ID(),
        onGeodeticWgs84PositionReading);

      double GPS_Heading{0};
      auto onGeodeticWgs84HeadingReading{[&GPS_Heading](
          cluon::data::Envelope &&envelope)
        {
          auto GPSHeadingReading = 
            cluon::extractMessage<opendlv::proxy::GeodeticWgs84HeadingReading>(
                std::move(envelope));
          GPS_Heading = GPSHeadingReading.northHeading();
        }};
      od4.dataTrigger(opendlv::proxy::GeodeticWgs84HeadingReading::ID(),
        onGeodeticWgs84HeadingReading);
      
      // cv::Mat templ = cv::imread("/home/opendlv/data/Project/team11-project-monorepo/robot-perception-service/Cone_blue.png",cv::IMREAD_GRAYSCALE );

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning()) {
        cv::Mat img;

        // Wait for a notification of a new frame.
        sharedMemory->wait();

        // Lock the shared memory.
        sharedMemory->lock();
        {
          cv::Mat wrapped(height, width, CV_8UC3, sharedMemory->data());
          img = wrapped.clone();
        }
        sharedMemory->unlock();
        // cv::imshow("Image: ", img);

        // Turn the color to HSV
        // cv::bitwise_not(img,img);
        // cv::imwrite("/home/opendlv/data/Project/team11-project-monorepo/robot-perception-service/ScreenShot.png",img);
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        cv::Rect roi(0, hsv.rows / 2 , hsv.cols, hsv.rows / 2 );
        // cv::Rect roi_center(hsv.cols / 4, hsv.rows * 5 / 9, hsv.cols / 2, hsv.rows /9);
        // hsv = hsv(roi);

        // Extract the yellow and blue cones out of the image
        cv::Mat yC_sharpen;
        cv::Mat bC_sharpen;
        cv::inRange(hsv(roi), cv::Scalar(20,195,110), cv::Scalar(30,255,255), yC_sharpen);    // For yellow cones
        cv::inRange(hsv(roi), cv::Scalar(80,0,0), cv::Scalar(180,200,60), bC_sharpen);  // For blue cones
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

        cv::Point2f AimPt = cv::Point2f((middlePt_yellowCone.x + middlePt_blueCone.x) / 2,(middlePt_yellowCone.y + middlePt_blueCone.y) / 2);
        float aimDirection = std::atan2(img.cols / 2 - AimPt.x,img.rows - AimPt.y); //Direction in rad

        if (verbose){
          cv::line(img,cv::Point2f(img.cols / 2,img.rows),AimPt,cv::Scalar(0,255,0));
          cv::line(img,cv::Point2f(img.cols / 2,img.rows - 10),cv::Point2f(AimPt.x,img.rows - 10),cv::Scalar(0,0,255));
          cv::putText(img, "Yaw Mag: " + std::to_string(yaw_mag / (2*std::acos(0.0)) * 180)
          , cv::Point(img.cols / 2,img.rows - 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0));
          cv::putText(img, "Yaw Gyro: " + std::to_string(yaw_gyro)
          , cv::Point(img.cols / 2,img.rows - 80), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0));
          cv::putText(img, "Velocity X: " + std::to_string(vX_acc)
          , cv::Point(img.cols / 2,img.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0));
          cv::putText(img, "Velocity Y: " + std::to_string(vY_acc)
          , cv::Point(img.cols / 2,img.rows - 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0));
          cv::putText(img, "Position Y: " + std::to_string(Y_acc)
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

