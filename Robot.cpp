#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <Talon.h>
#include "WPILib.h"
#include <XboxController.h> //gamepad
#include <Spark.h>


//Vision
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <thread>

class Robot: public frc::IterativeRobot {
public:
	Robot() {
		timer.Start();
	}

private:
	frc::RobotDrive *myRobot;  // Robot drive system
	frc::Joystick *stick = new Joystick(0);
	frc::LiveWindow *lw = frc::LiveWindow::GetInstance();
	frc::Timer timer;
	frc::Talon *frontLeft, *frontRight, *rearRight;
	frc::Spark *rearLeft;
	//frc::XboxController *gamepad = new XboxController(0);


	static void VisionThread()
	{
		// Get the USB camera from CameraServer
		cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
		// Set the resolution
		camera.SetResolution(1280, 720);
		camera.SetFPS(30);
		// Get a CvSink. This will capture Mats from the Camera
		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
		// Setup a CvSource. This will send images back to the Dashboard
		cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Rectangle", 1280, 720);


		// Mats are very memory expensive. Lets reuse this Mat.
		cv::Mat mat;

		while (true) {
			// Tell the CvSink to grab a frame from the camera and put it
			// in the source mat.  If there is an error notify the output.
			if (cvSink.GrabFrame(mat) == 0) {
				// Send the output the error.
				outputStream.NotifyError(cvSink.GetError());
				// skip the rest of the current iteration
				continue;
			}
			// Put a rectangle on the image
			rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),cv::Scalar(255, 255, 255), 5);

			// Give the output stream a new image to display
			outputStream.PutFrame(mat);
			}

	}

	void RobotInit() override {
		std::thread visionThread(VisionThread);
		visionThread.detach();

		frontLeft = new Talon(0);
		frontRight = new Talon(2);
		rearLeft = new Spark(3);
		rearRight = new Talon(1);
		myRobot = new RobotDrive(frontLeft,frontRight,rearLeft,rearRight);
		myRobot->SetExpiration(0.1);
	}

	void AutonomousInit() override {
		timer.Reset();
		timer.Start();
	}

	void AutonomousPeriodic() override {
		// Drive for 2 seconds
		if (timer.Get() < 2.0) {
			myRobot->Drive(-0.5, 0.0);  // Drive forwards half speed
			//get gyro so it will drive straight all the time
		} else {
			myRobot->Drive(0.0, 0.0);  // Stop robot
		}
	}

	void TeleopInit() override {

	}

	void TeleopPeriodic() override {
		// Drive with arcade style (use right stick)
		myRobot->ArcadeDrive(stick);
		//Mecanum drive may be best here
	}

	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)


