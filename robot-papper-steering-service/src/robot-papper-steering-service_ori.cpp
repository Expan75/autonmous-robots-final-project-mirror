#include <cmath>
#include <cstdint>
#include <iostream>

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"

int32_t main(int32_t argc, char **argv)
{
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if (cmd.count("cid") == 0
      || 0 == cmd.count("freq")) {
    std::cout << argv[0] << " is an OpenDLV microservice." << std::endl;
    std::cout << "Usage: " << argv[0] << " "
              << "--cid=<conference id; e.g. 111> "
              << "[--verbose] " << std::endl;
    return 0;
  }

  uint16_t const cid = std::stoi(cmd.at("cid"));

  bool const verbose = (cmd.count("verbose") != 0);
  if (cid != 111) {
    std::cout << cid << " is not an valid conference ID number." << std::endl;
    return 0;
  }
  if (verbose) {
    std::cout << "Starting microservice." << std::endl;
  }
  float const FREQ = std::stof(cmd["freq"]);
  float reference_Vx = (cmd.count("refVx") != 0) ? std::stof(cmd["refVx"]) : 0.6f; // unit: m/s
  float const DT = 1.0f / FREQ;

  cluon::OD4Session od4(cid);

  // Try to catch the position of the object and send out the acceleration and the turning angle
  float vX_measured{0.0f};
  float vY_measured{0.0f};
  float Y_measured{0.0f};
  float yaw_dot_measured{0.0f};
  auto onDetectionProperty{[&vX_measured, &vY_measured, &Y_measured, &yaw_dot_measured](cluon::data::Envelope &&envelope) {
    auto const Dp =
        cluon::extractMessage<opendlv::logic::perception::DetectionProperty>(
            std::move(envelope));

    std::string strPosition = Dp.property();
    size_t comma_index = strPosition.find(";");
    size_t comma_index_1 = strPosition.substr(comma_index+1,strPosition.length()-1).find(";");
    vX_measured = std::stof(strPosition.substr(0,comma_index));
    vY_measured = std::stof(strPosition.substr(comma_index+1,comma_index_1));
    comma_index = strPosition.substr(comma_index_1+1,strPosition.length()-1).find(";");
    Y_measured = std::stof(strPosition.substr(comma_index_1+1,comma_index));
    yaw_dot_measured = std::stof(strPosition.substr(comma_index+1,strPosition.length()-1));;
  }};
  od4.dataTrigger(opendlv::logic::perception::DetectionProperty::ID(), onDetectionProperty);

  float aimDirection_cur{0.0f};
  float aimDirection_dev{0.0f};
  auto onAimDirectionReading{[&aimDirection, &aimDirection_dot, &DT](
      cluon::data::Envelope &&envelope)
    {
      auto ADReading = 
        cluon::extractMessage<opendlv::logic::action::AimDirection>(
            std::move(envelope));
      float aimDirection_new = ADReading.azimuthAngle();
      aimDirection_dot = (aimDirection_new - aimDirection);
      // std::cout << "Angle: " << std::to_string(aimDirection_new) << std::endl;
      // std::cout << "Angle dot: " << std::to_string(aimDirection_dot) << std::endl;
      aimDirection = aimDirection_new;
    }};
  od4.dataTrigger(opendlv::logic::action::AimDirection::ID(),
    onAimDirectionReading);

  // auto onFrameReading{[&yaw_model](
  //     cluon::data::Envelope &&envelope)
  //   {
  //     auto FrReading = 
  //       cluon::extractMessage<opendlv::sim::Frame>(
  //           std::move(envelope));
  //     yaw_model = std::atan2(2.0f*(FrReading.qy()*FrReading.qz() + FrReading.qw()*FrReading.qx())
  //     , FrReading.qw()*FrReading.qw() - FrReading.qx()*FrReading.qx() - FrReading.qy()*FrReading.qy() + FrReading.qz()*FrReading.qz());
  //   }};
  // od4.dataTrigger(opendlv::sim::Frame::ID(),
  //   onFrameReading);
    
  float vX_estimate{0.0f};
  float vY_estimate{0.0f};
  float yaw_dot_estimate{0.0f};
  float Y_estimate{0.0f};
  auto onKinematicStateReading{[&vX_estimate, &vY_estimate, &Y_estimate, &yaw_dot_estimate, &DT](
      cluon::data::Envelope &&envelope)
    {
      auto KSReading = 
        cluon::extractMessage<opendlv::sim::KinematicState>(
            std::move(envelope));
      vX_estimate = KSReading.vx();
      vY_estimate = KSReading.vy();
      yaw_dot_estimate = KSReading.yawRate();
      Y_estimate += vY_estimate*DT; // 0.01 is for the sampling time
    }};
  od4.dataTrigger(opendlv::sim::KinematicState::ID(),
    onKinematicStateReading);

  float steering_template{0.0f};
  // Data trigger lambda functions.
  auto onGroundSteeringRequest{[&steering_template](
      cluon::data::Envelope &&envelope)
    {
      auto groundSteeringAngleRequest = 
        cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(
            std::move(envelope));
      steering_template = groundSteeringAngleRequest.groundSteering();
    }};

  float pedal_template{0.0f};
  auto onPedalPositionRequest{[&pedal_template](
      cluon::data::Envelope &&envelope)
    {
      auto pedalPositionRequest = 
        cluon::extractMessage<opendlv::proxy::PedalPositionRequest>(
            std::move(envelope));
      pedal_template = pedalPositionRequest.position();
    }};

  od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(),
      onGroundSteeringRequest);
  od4.dataTrigger(opendlv::proxy::PedalPositionRequest::ID(),
      onPedalPositionRequest);
  
  auto atFrequency{[&od4, &vX_measured, &vY_measured, &Y_measured, &yaw_dot_measured
  , & vX_estimate, &vY_estimate, &yaw_dot_estimate, &Y_estimate, &reference_Vx, &aimDirection_dot
  , &steering_template, &pedal_template]() 
  -> bool
  {
    std::mutex Mutex;
    // float vX_measured_copy, vY_measured_copy, Y_measured_copy, yaw_dot_measured_copy;
    float vX_estimate_copy, vY_estimate_copy, yaw_dot_estimate_copy;
    float aimDirection_dot_copy;
    // {
    //   std::lock_guard<std::mutex> lock1(Mutex);
    //   vX_measured_copy = vX_measured;
    // }
    // {
    //   std::lock_guard<std::mutex> lock2(Mutex);
    //   vY_measured_copy = vY_measured;
    // }
    // {
    //   std::lock_guard<std::mutex> lock3(Mutex);
    //   Y_measured_copy = Y_measured;
    // }
    // {
    //   std::lock_guard<std::mutex> lock4(Mutex);
    //   yaw_dot_measured_copy = yaw_dot_measured;
    // }
    {
      std::lock_guard<std::mutex> lock5(Mutex);
      vX_estimate_copy = vX_estimate;
    }
    {
      std::lock_guard<std::mutex> lock6(Mutex);
      vY_estimate_copy = vY_estimate;
    }
    // {
    //   std::lock_guard<std::mutex> lock7(Mutex);
    //   Y_estimate_copy = Y_estimate;
    // }
    {
      std::lock_guard<std::mutex> lock8(Mutex);
      yaw_dot_estimate_copy = yaw_dot_estimate;
    }
    {
      std::lock_guard<std::mutex> lock9(Mutex);
      aimDirection_dot_copy = aimDirection_dot;
    }

    // Do Kalman Filter
    float vX_corr, vY_corr, yaw_dot_corr;
    // vX_corr = vX_estimate_copy + 0.0122f * (vX_measured_copy - vX_estimate_copy);
    // vY_corr = vY_estimate_copy + 0.0015f * (vY_measured_copy - vY_estimate_copy) - 0.0022f * (yaw_dot_measured_copy - yaw_dot_estimate_copy);
    // yaw_dot_corr = yaw_dot_estimate_copy + 0.9901f * (yaw_dot_measured_copy - yaw_dot_estimate_copy);
    // Y_corr = Y_estimate_copy - 0.0001f * (yaw_dot_measured_copy - yaw_dot_estimate_copy) + 0.0951f * (Y_measured_copy - Y_estimate_copy);
    vX_corr = vX_estimate_copy ;
    vY_corr = vY_estimate_copy;
    yaw_dot_corr = yaw_dot_estimate_copy;
    // Y_corr = Y_estimate_copy;

    // std::cout << "vX: " << std::to_string(vX_corr) 
    //           << ", vY: " << std::to_string(vY_corr)
    //           << ", yaw_dot: " << std::to_string(yaw_dot_corr) << std::endl;

    // Calculate the control signal here (LQR or PID)
    float pedal_control = 2.0909f * vX_corr;
    float steering_control = 3.0175f * vY_corr - 0.0261f * yaw_dot_corr;

    float vX_ref{0.0f}, yaw_dot_ref{0.0f};
    vX_ref = reference_Vx;
    yaw_dot_ref = 1.8402f * aimDirection_dot_copy;


    // std::cout << "Aim Direction dot: " << std::to_string(aimDirection_dot)  << std::endl;

    float pedal_ref = 2.2447f * vX_ref;
    float steering_ref = 0.4240f * yaw_dot_ref;

    float pedal_opt = pedal_ref - pedal_control;
    float steering_opt = steering_ref - steering_control;
    steering_opt = steering_opt;

    pedal_opt = 0.45f + (-pedal_opt + 0.15f) * 0.45f / 1.45f;
    // steering_opt = 1.0f * steering_opt;
    std::cout << "Reference steering: " << std::to_string(steering_ref) << std::endl;
    // std::cout << "Aim direction yaw_dot: " << std::to_string(aimDirection_dot_copy) << std::endl;
    // std::cout << "Control steering: " << std::to_string(steering_control) << std::endl;
    // std::cout << ", measured vY: " << std::to_string(vY_corr) << std::endl;
    // std::cout << ", measured yaw_dot: " << std::to_string(yaw_dot_corr) << std::endl;

    // std::cout << "Optimal pedal position: " << std::to_string(pedal_opt) 
    // std::cout << ", optimal steering: " << std::to_string(steering_opt) << std::endl;
    // std::cout << "Template pedal position: " << std::to_string(pedal_template) 
    //           << ", template steering: " << std::to_string(steering_template) << std::endl;

    // Send the control request to other microservice
    // opendlv::proxy::GroundSteeringRequest gsr;
    // gsr.groundSteering(steering_opt);
    // od4.send(gsr);

    // // Uncomment the following lines to accelerate/decelerate; range: +0.25
    // // (forward) .. -1.0 (backwards). Be careful!
    // opendlv::proxy::PedalPositionRequest ppr;
    // ppr.position(pedal_opt);
    // od4.send(ppr); 

    return true;
  }};

  // Will run until Ctrl+C is pressed.
  od4.timeTrigger(FREQ, atFrequency); 

  return 0;
}
