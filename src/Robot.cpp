/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include "ctre/Phoenix.h"

class Robot : public frc::TimedRobot
{
public:
	void RobotInit() override
	{
		try
		{
			LeftElevatorTalon.reset(new WPI_TalonSRX(10));
		}
		catch(const std::exception& e)
		{
			printf("There be dragons here");
		}
		int absolutePosition = LeftElevatorTalon->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		/* use the low level API to set the quad encoder signal */
		LeftElevatorTalon->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

		/* choose quadrature which has a faster update rate */
		LeftElevatorTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, kTimeoutMs);
		//LeftElevatorTalon->SetStatusFramePeriod(StatusFrame::Status_1_General_, 10, kTimeoutMs);
		//LeftElevatorTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 10, kTimeoutMs);

		LeftElevatorTalon->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		LeftElevatorTalon->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		LeftElevatorTalon->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
		LeftElevatorTalon->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);

		LeftElevatorTalon->SetSensorPhase(kSensorPhase);

		LeftElevatorTalon->ConfigNominalOutputForward(0, kTimeoutMs);
		LeftElevatorTalon->ConfigNominalOutputReverse(0, kTimeoutMs);
		LeftElevatorTalon->ConfigPeakOutputForward(kMaxElevatorSpeed, kTimeoutMs);
		LeftElevatorTalon->ConfigPeakOutputReverse(-kMaxElevatorSpeed, kTimeoutMs);

		SmartDashboard::PutNumber("DB/Slider 0", kP);
		SmartDashboard::PutNumber("DB/Slider 1", kI);
		SmartDashboard::PutNumber("DB/Slider 2", kD);
		SmartDashboard::PutNumber("DB/Slider 3", kF);
	}

	/**
	 * This function is called once each time the robot enters Disabled
	 * mode.
	 * You can use it to reset any subsystem information you want to clear
	 * when
	 * the robot is disabled.
	 */
	void DisabledInit() override {}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to
	 * the
	 * chooser code above (like the commented example) or additional
	 * comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {
		std::string autoSelected = frc::SmartDashboard::GetString(
				"Auto Selector", "Default");
		/*
		if (autoSelected == "My Auto") {
			m_autonomousCommand = &m_myAuto;
		} else {
			m_autonomousCommand = &m_defaultAuto;
		}

		m_autonomousCommand = m_chooser.GetSelected();

		if (m_autonomousCommand != nullptr) {
			m_autonomousCommand->Start();
		}
		*/
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != nullptr) {
			m_autonomousCommand->Cancel();
			m_autonomousCommand = nullptr;
		}
	}

	void TeleopPeriodic() override
	{
		if(desiredPosition != stoi(SmartDashboard::GetString("DB/String 0", "0.0")))
		{
			desiredPosition = stoi(SmartDashboard::GetString("DB/String 0", "0.0"));
		}

		if(kP != SmartDashboard::GetNumber("DB/Slider 0", 0.0))
		{
			kP = SmartDashboard::GetNumber("DB/Slider 0", kP);
			LeftElevatorTalon->Config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		}

		if(kI != SmartDashboard::GetNumber("DB/Slider 1", 0.0))
		{
			kI = SmartDashboard::GetNumber("DB/Slider 1", kI);
			LeftElevatorTalon->Config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		}

		if(kD != SmartDashboard::GetNumber("DB/Slider 2", 0.0))
		{
			kD = SmartDashboard::GetNumber("DB/Slider 2", kD);
			LeftElevatorTalon->Config_kD(kPIDLoopIdx, kD, kTimeoutMs);
		}

		if(kF != SmartDashboard::GetNumber("DB/Slider 3", 0.0))
		{
			kF = SmartDashboard::GetNumber("DB/Slider 3", kF);
			LeftElevatorTalon->Config_kF(kPIDLoopIdx, kF, kTimeoutMs);
		}

		LeftElevatorTalon->Set(ControlMode::Position, desiredPosition);
		SmartDashboard::PutString("DB/String 5", std::to_string(fabs(LeftElevatorTalon->GetSensorCollection().GetQuadraturePosition())));
		frc::Scheduler::GetInstance()->Run();
	}

	void TestPeriodic() override {}

private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
	frc::Command* m_autonomousCommand = nullptr;
	frc::SendableChooser<frc::Command*> m_chooser;
	std::shared_ptr<WPI_TalonSRX> LeftElevatorTalon;
	int desiredPosition = 0;
	double kP = 0.1;
	double kI = 0.0;
	double kD = 0.0;
	double kF = 0.0;
	static constexpr bool kSensorPhase = false;
	static constexpr int kSlotIdx = 0;
	static constexpr int kPIDLoopIdx = 0;
	static constexpr int kTimeoutMs = 10;
	static constexpr int kNoTimeoutMs = 0;
	static constexpr double kMaxElevatorSpeed = 0.99;
};

START_ROBOT_CLASS(Robot)
