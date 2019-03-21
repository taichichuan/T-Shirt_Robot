/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/XboxController.h>
#include <frc/SpeedController.h>
#include <frc/SpeedControllerGroup.h>
#include <ctre/Phoenix.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>


/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */

using namespace frc;
using namespace std;
using namespace nt;

class Robot : public frc::TimedRobot
{
  // CAN IDs
  const static int kPdpCanAddress = 0;

  // Channels for the wheels  (CAN IDs)
  const static int frontLeftChannel = 1;
  const static int rearLeftChannel = 2;
  const static int frontRightChannel = 3;
  const static int rearRightChannel = 4;

  // Miscellaneous constants
  const float Kp = 0.03;
  const double kUpdatePeriod = 0.010; // update period in seconds

  // Joystick port assignments
  const static int kJoystickChannel0 = 0;

  // Joystick deadband settings
  const float kBottomOfDeadBand = -0.1;
  const float kTopOfDeadBand = 0.1;

  // Joystick buttons
  const int kFullSpeed = 6; // right bumper button on Xbox Controller

  // Mobility 4WD
  WPI_TalonSRX *m_frontLeft = new WPI_TalonSRX(frontLeftChannel);
  WPI_TalonSRX *m_rearLeft = new WPI_TalonSRX(rearLeftChannel);
  WPI_TalonSRX *m_frontRight = new WPI_TalonSRX(frontRightChannel);
  WPI_TalonSRX *m_rearRight = new WPI_TalonSRX(rearRightChannel);

  SpeedControllerGroup m_left{*m_frontLeft, *m_rearLeft};
  SpeedControllerGroup m_right{*m_frontRight, *m_rearRight};
  DifferentialDrive m_robotDrive{m_left, m_right};
  XboxController m_stick{kJoystickChannel0};

public:

  void RobotInit() override {
    // We need to run our vision program in a separate thread. If not, our robot
    // program will not run.
    std::thread visionThread(VisionThread);
    visionThread.detach();
  }

  void TeleopPeriodic()
  {
    float rightY;
    float leftY;
    bool fullSpeed;

    // Drive with arcade style

    // Implement the deadband
    rightY = -m_stick.GetY(frc::GenericHID::kRightHand);
    leftY = -m_stick.GetY(frc::GenericHID::kLeftHand);

    if ((rightY > kBottomOfDeadBand) and (rightY < kTopOfDeadBand))
      rightY = 0;

    if ((leftY > kBottomOfDeadBand) and (leftY < kTopOfDeadBand))
      leftY = 0;

    // cube the input to shape it while preserving sign
    rightY = rightY * rightY * rightY;
    leftY = leftY * leftY * leftY;

    fullSpeed = m_stick.GetRawButton(kFullSpeed);

    if (fullSpeed)
    {
      // Make it so....
      m_robotDrive.TankDrive(leftY, rightY);
    }
    else
    {
      m_robotDrive.TankDrive(leftY * 0.75, rightY * 0.75);
    }
  }

private:
  static void VisionThread()
  {
    cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(160, 120); // Microsoft LifeCam
    camera.SetFPS(15);
    camera.SetVideoMode(cs::VideoMode::PixelFormat::kMJPEG, 640, 480, 30);
/*
    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;

    while (true)
    {
      // Tell the CvSink to grab a frame from the camera and put it in the
      // source mat. If there is an error notify the output.
      if (cvSink.GrabFrame(mat) == 0)
      {
        // Send the output the error.
        outputStream.NotifyError(cvSink.GetError());
        // skip the rest of the current iteration
        continue;
      }
      // Put a rectangle on the image
      rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
                cv::Scalar(255, 255, 255), 5);
      // Give the output stream a new image to display
      outputStream.PutFrame(mat);
    }
    */
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
