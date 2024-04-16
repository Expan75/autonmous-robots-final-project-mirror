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
        double alpha = 1.7; /*< Simple contrast control */
        double beta = 3; /*< Simple brightness control */
        cv::Mat sharpenImage;
        img.convertTo(sharpenImage, -1, alpha, beta); 

        // Turn the color to HSV
        cv::Mat hsv;
        cv::cvtColor(sharpenImage, hsv, cv::COLOR_BGR2HSV);

        // Only focus on the bottom part of the image
        cv::Rect roi(0, hsv.rows / 2 , hsv.cols, hsv.rows / 2 );
        cv::Mat bottom_half = hsv(roi);
        cv::Mat kernel = (cv::Mat_<int>(3,3) << 1, 1, 1, 1, -8, 1, 1, 1, 1);

        // Distinguish yellow and blue color from the image
        // Find yellow color
        cv::Scalar hsvLow(20, 50, 50);
        cv::Scalar hsvHigh(50, 255, 255);
        cv::Mat yellowCones;
        cv::inRange(bottom_half, hsvLow, hsvHigh, yellowCones);   

        // Extract the yellow color and transform to gray color to control by threshold
        cv::Mat tempMat;
        bottom_half.copyTo(tempMat,yellowCones);
        cv::cvtColor(tempMat, yellowCones, cv::COLOR_BGR2GRAY );
        // cv::GaussianBlur( Conesimg_gray, Conesimg_blur, cv::Size(5,5), 0 );
        cv::threshold(yellowCones, yellowCones, 40, 255, cv::THRESH_TOZERO );
        cv::dilate(yellowCones,yellowCones,kernel, cv::Point(-1,-1),3,1,1);
        // cv::imshow("Yellow Cone Image", yellowCones);
        
        // Find blue color
        cv::Scalar bhsvLow(100, 50, 50);
        cv::Scalar bhsvHigh(150, 255, 255);
        cv::Mat blueCones;
        cv::inRange(bottom_half, bhsvLow, bhsvHigh, blueCones); 

        // Extract the blue color and transform to gray color to control by threshold  
        cv::Mat tempMat_b; 
        bottom_half.copyTo(tempMat_b,blueCones);
        cv::cvtColor(tempMat_b, blueCones, cv::COLOR_BGR2GRAY );
        // cv::GaussianBlur( Conesimg_gray, Conesimg_blur, cv::Size(5,5), 0 );
        cv::threshold(blueCones, blueCones, 115, 255, cv::THRESH_TOZERO );
        cv::dilate(blueCones,blueCones,kernel, cv::Point(-1,-1),3,1,1);
        // cv::imshow("Blue Cone Image", blueCones);

        // Find contour for the cones
        std::vector<std::vector<cv::Point>> contours_blue, contours_yellow;
        cv::findContours(blueCones, contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(yellowCones, contours_yellow, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Find center point of the contours
        std::vector<cv::Point2f> cPArray_b;
        std::vector<cv::Point2f> cPArray_y;

        // Dealing with yellow cones first
        for ( std::size_t i = 0; i < contours_yellow.size(); i++ )
        {
            cv::Moments M = cv::moments(contours_yellow[i]);
            float center_x = static_cast<float>(M.m10 / (M.m00 + 1e-5));
            float center_y = static_cast<float>(M.m01 / (M.m00 + 1e-5));

            // Ignore car
            if ( ( center_x > hsv.cols / 5 && hsv.cols - center_x > hsv.cols / 5 ) && hsv.rows / 2 - center_y <= hsv.rows / 7 ){
              continue;
            }

            // Put the point into the vector
            cPArray_y.push_back(cv::Point2f(center_x, center_y ));
        }
        std::sort(cPArray_y.begin(), cPArray_y.end(), comparePointx); // Sort the vector for later use
        
        // Dealing with blue cones first
        for ( std::size_t i = 0; i < contours_blue.size(); i++ )
        {
            cv::Moments M = cv::moments(contours_blue[i]);
            float center_x = static_cast<float>(M.m10 / (M.m00 + 1e-5));
            float center_y = static_cast<float>(M.m01 / (M.m00 + 1e-5));

            // Ignore car
            if ( ( center_x > hsv.cols / 5 && hsv.cols - center_x > hsv.cols / 5 ) && hsv.rows / 2 - center_y <= hsv.rows / 7 ){
              continue;
            }

            // put the point into the vector
            cPArray_b.push_back(cv::Point2f(center_x, center_y ));
        }
        std::sort(cPArray_b.begin(), cPArray_b.end(), comparePointx); // Sort the vector for later use

        // Process two vector, mainly to further remove outliers
        // Yellow cones part
        std::vector<cv::Point2f> cPArray_y_temp;
        for ( std::size_t i = 0; i < cPArray_y.size(); i++ )
        {
          // Ignore outlier origin
          if (i == 0 && cPArray_y[0].x > hsv.cols / 2)
          {
            continue;
          }          

          // Ignore outlier yellow cones in wrong position
          if ( cPArray_b.size() > 0 && cPArray_y[i].x > hsv.cols *3 / 4 )
          {
            cv::circle( img, cv::Point2f(cPArray_y[i].x, cPArray_y[i].y + img.rows/2), 5, cv::Scalar(0, 0, 255), cv::FILLED); //red point for point which were filtered out
            continue;
          }

          // Ignore too far away outliers
          if ( cPArray_y.size() > 1 && std::abs(cPArray_y[i].x - cPArray_y[i - 1].x) > hsv.cols / 3  )
          {
            cv::circle( img, cv::Point2f(cPArray_y[i].x, cPArray_y[i].y + img.rows/2), 5, cv::Scalar(0, 0, 255), cv::FILLED); //red point for point which were filtered out
            continue;
          }

          // Push the process point to the new array
          cv::circle( img, cv::Point2f(cPArray_y[i].x, cPArray_y[i].y + img.rows/2), 5, cv::Scalar(0, 255, 0), cv::FILLED); //green point for point which were not filtered out
          cPArray_y_temp.push_back(cPArray_y[i]);
        }
        
        // Blue cones part
        std::vector<cv::Point2f> cPArray_b_temp;
        for ( std::size_t i = 0; i < cPArray_b.size(); i++ )
        {
          // Ignore outlier origin && blue cones in wrong position
          if ( cPArray_b[i].x < hsv.cols / 2 )
          {
            cv::circle( img, cv::Point2f(cPArray_b[i].x, cPArray_b[i].y + img.rows/2), 5, cv::Scalar(0, 0, 255), cv::FILLED); //red point for point which were filtered out
            continue;
          }

          // Ignore too far away outliers
          if ( cPArray_b.size() > 1 && std::abs(cPArray_b[i].x - cPArray_b[i - 1].x) > hsv.cols / 4  )
          {
            cv::circle( img, cv::Point2f(cPArray_b[i].x, cPArray_b[i].y + img.rows/2), 5, cv::Scalar(0, 0, 255), cv::FILLED); //red point for point which were filtered out
            continue;
          }

          // Push the process point to the new array
          cv::circle( img, cv::Point2f(cPArray_b[i].x, cPArray_b[i].y + img.rows/2), 5, cv::Scalar(0, 255, 0), cv::FILLED); //green point for point which were not filtered out
          cPArray_b_temp.push_back(cPArray_b[i]);
        }

        // Sort two vectors for later use
        std::sort(cPArray_b_temp.begin(), cPArray_b_temp.end(), comparePointy);        
        std::sort(cPArray_y_temp.begin(), cPArray_y_temp.end(), comparePointy);

        // Send message to another microservice
        opendlv::logic::perception::DetectionProperty message;
        for ( std::size_t i = 0; i < cPArray_b_temp.size(); i++ )
        {
          message.sampleId(i); //0 stands for the first point
          message.detectionId(1); //0 stands for yellow cone, 1 for the blue cone
          message.property(std::to_string(cPArray_b_temp[i].x) + ";" + std::to_string(cPArray_b_temp[i].y)); //property stands for the position
          od4.send(message);
        }
        for ( std::size_t i = 0; i < cPArray_y_temp.size(); i++ )
        {
          message.sampleId(i); //0 stands for the first point
          message.detectionId(0); //0 stands for yellow cone, 1 for the blue cone
          message.property(std::to_string(cPArray_y_temp[i].x) + ";" + std::to_string(cPArray_y_temp[i].y)); //property stands for the position
          od4.send(message);
        }

        /////////////////Steering Algorithm Part/////////////////////////
        // Follow part might be dealed with another microservice later
        // Find the first and the last point and find the final aim point
        cv::Point2f yellowConeFpt, yellowConeLpt, blueConeFpt, blueConeLpt;
        cv::Point2f tempFpt, tempLpt, AimPt, CenterPt;
        if ( cPArray_y_temp.size() > 0 && cPArray_b_temp.size() > 0 )
        {
          yellowConeFpt = cv::Point2f(cPArray_y_temp[0].x, cPArray_y_temp[0].y + hsv.rows / 2);
          yellowConeLpt = cv::Point2f(cPArray_y_temp[cPArray_y_temp.size()-1].x, cPArray_y_temp[cPArray_y_temp.size()-1].y + hsv.rows / 2);
          blueConeFpt = cv::Point2f(cPArray_b_temp[0].x, cPArray_b_temp[0].y + hsv.rows / 2);
          blueConeLpt = cv::Point2f(cPArray_b_temp[cPArray_b_temp.size()-1].x, cPArray_b_temp[cPArray_b_temp.size()-1].y + hsv.rows / 2);
        }
        else if ( cPArray_y_temp.size() > 0 && cPArray_b_temp.size() == 0 )
        {
          yellowConeFpt = cv::Point2f(cPArray_y_temp[0].x, cPArray_y_temp[0].y + hsv.rows / 2);
          yellowConeLpt = cv::Point2f(cPArray_y_temp[cPArray_y_temp.size()-1].x, cPArray_y_temp[cPArray_y_temp.size()-1].y + hsv.rows / 2);
          blueConeFpt = cv::Point2f(hsv.cols, hsv.rows);
          blueConeLpt = cv::Point2f(hsv.cols / 2, hsv.rows);
        }
        else if( cPArray_y_temp.size() == 0 && cPArray_b_temp.size() > 0 )
        {  
          yellowConeFpt = cv::Point2f(0, hsv.rows);
          yellowConeLpt = cv::Point2f(hsv.cols / 2, hsv.rows);
          blueConeFpt = cv::Point2f(cPArray_b_temp[0].x, cPArray_b_temp[0].y + hsv.rows / 2);
          blueConeLpt = cv::Point2f(cPArray_b_temp[cPArray_b_temp.size()-1].x, cPArray_b_temp[cPArray_b_temp.size()-1].y + hsv.rows / 2);   
        }
        tempFpt = cv::Point2f((yellowConeFpt.x + blueConeFpt.x) / 2, (yellowConeFpt.y + blueConeFpt.y) / 2);
        tempLpt = cv::Point2f((yellowConeLpt.x + blueConeLpt.x) / 2, (yellowConeLpt.y + blueConeLpt.y) / 2);
        if ( cPArray_y_temp.size() == 0 )
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
          cv::imshow("Process Result", ResultMat);
          // cv::imshow(sharedMemory->name().c_str(), ResultMat);
          cv::waitKey(1);
        }

        ////////////////////////////////////////////////////////////////
        // Do something with the distance readings if wanted.
        {
          std::lock_guard<std::mutex> lck(distancesMutex);
          std::cout << "front = " << front << ", "
                    << "rear = " << rear << ", "
                    << "left = " << left << ", "
                    << "right = " << right << "." << std::endl;
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

