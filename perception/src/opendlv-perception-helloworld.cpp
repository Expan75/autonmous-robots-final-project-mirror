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

        // Invert colors
       // cv::bitwise_not(img, img);

        // Draw a red rectangle
       // cv::rectangle(
       //     img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0, 0, 255));


        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar hsvLow(110, 50, 50);
        cv::Scalar hsvHigh(130, 255, 255);
        cv::Mat blueCones;
        cv::inRange(hsv, hsvLow, hsvHigh, blueCones);
        cv::imshow("Blue cones", blueCones);


        // Display image.
        if (verbose) {
          cv::imshow(sharedMemory->name().c_str(), img);
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
        // opendlv::proxy::GroundSteeringRequest gsr;
        // gsr.groundSteering(0);
        // od4.send(gsr);

        // Uncomment the following lines to accelerate/decelerate; range: +0.25
        // (forward) .. -1.0 (backwards). Be careful!
        // opendlv::proxy::PedalPositionRequest ppr;
        // ppr.position(0);
        // od4.send(ppr);
      }
    }
    retCode = 0;
  }
  return retCode;
}

