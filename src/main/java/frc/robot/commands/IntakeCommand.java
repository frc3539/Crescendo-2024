// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.utilities.BBMath;

public class IntakeCommand extends Command {
public enum IntakeMode {
	FRONT,
	BACK,
	SENSOR;
}

boolean intake;
IntakeMode mode;
IntakeMode initialMode;
Timer intakeTimer = new Timer();
boolean timerStarted = false;

// Creates a new IntakeCommand.
public IntakeCommand(boolean intake, IntakeMode mode) {
	addRequirements(RobotContainer.intakeSubsystem, RobotContainer.shooterSubsystem);
	this.intake = intake;
	this.mode = mode;
	this.initialMode = mode;
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
	if (RobotContainer.shooterSubsystem.getShooterSensor() && intake == true) {
	RobotContainer.intakeSubsystem.setGroundMotorVoltage(0);
	RobotContainer.intakeSubsystem.setChamberMotorVoltage(0);
	RobotContainer.intakeSubsystem.setKickMotorVoltage(0);
	RobotContainer.shooterSubsystem.setFeedMotorVoltage(0);

	return;
	}
	if (intake == true) {
	// RobotContainer.intakeSubsystem.setGroundMotorSpeed(IntakeConstants.intakeRps);
	// RobotContainer.intakeSubsystem.setGrabMotorSpeed(IntakeConstants.intakeRps);

	RobotContainer.intakeSubsystem.setGroundMotorSpeed(
		BBMath.getRps(IntakeConstants.intakeDps * 3, IntakeConstants.groundWheelDiameter));
	RobotContainer.intakeSubsystem.setChamberMotorSpeed(
		BBMath.getRps(IntakeConstants.intakeDps, IntakeConstants.chamberWheelDiameter));
	RobotContainer.shooterSubsystem.setFeedMotorSpeed(
		BBMath.getRps(IntakeConstants.intakeDps, ShooterConstants.feedWheelDiameter));

	} else {
	// RobotContainer.intakeSubsystem.setGroundMotorSpeed(-IntakeConstants.intakeRps);
	// RobotContainer.intakeSubsystem.setGrabMotorSpeed(-IntakeConstants.intakeRps);

	RobotContainer.intakeSubsystem.setGroundMotorSpeed(
		BBMath.getRps(-IntakeConstants.intakeDps * 3, IntakeConstants.groundWheelDiameter));
	RobotContainer.intakeSubsystem.setChamberMotorSpeed(
		BBMath.getRps(-IntakeConstants.intakeDps, IntakeConstants.chamberWheelDiameter));
	RobotContainer.shooterSubsystem.setFeedMotorSpeed(
		BBMath.getRps(-IntakeConstants.intakeDps, ShooterConstants.feedWheelDiameter));
	}
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
	// if (RobotContainer.shooterSubsystem.getShooterSensor() && intake == true) {
	// RobotContainer.intakeSubsystem.setGroundMotorVoltage(0);
	// RobotContainer.intakeSubsystem.setChamberMotorVoltage(0);
	// RobotContainer.intakeSubsystem.setKickMotorVoltage(0);
	// RobotContainer.shooterSubsystem.setFeedMotorVoltage(0);
	// return;
	// }

	double multiplier = 1;
	if (RobotContainer.intakeSubsystem.getChamberSensor()) {
	multiplier = 1;
	RobotContainer.intakeSubsystem.setGroundMotorVoltage(0);
	}

	if (RobotContainer.intakeSubsystem.getChamberSensor() && intake == true) {
	RobotContainer.intakeSubsystem.setChamberMotorSpeed(
		multiplier
			* BBMath.getRps(IntakeConstants.intakeDps, IntakeConstants.chamberWheelDiameter));
	RobotContainer.shooterSubsystem.setFeedMotorSpeed(
		multiplier
			* BBMath.getRps(IntakeConstants.intakeDps, ShooterConstants.feedWheelDiameter));
	}

	switch (mode) {
	case FRONT:
		// RobotContainer.intakeSubsystem.setKickMotorSpeed(IntakeConstants.kickRps);
		RobotContainer.intakeSubsystem.setKickMotorSpeed(
			multiplier
				* BBMath.getRps(
					IntakeConstants.intakeDps / IntakeConstants.kickGearRatio,
					IntakeConstants.kickWheelDiameter));
		break;

	case BACK:
		// RobotContainer.intakeSubsystem.setKickMotorSpeed(-IntakeConstants.kickRps);
		RobotContainer.intakeSubsystem.setKickMotorSpeed(
			multiplier
				* -BBMath.getRps(
					IntakeConstants.intakeDps / IntakeConstants.kickGearRatio,
					IntakeConstants.kickWheelDiameter));
		break;

	case SENSOR:
		if (RobotContainer.intakeSubsystem.getFrontSensor()) {
		RobotContainer.intakeSubsystem.setKickMotorSpeed(
			multiplier
				* BBMath.getRps(
					IntakeConstants.intakeDps / IntakeConstants.kickGearRatio,
					IntakeConstants.kickWheelDiameter));
		this.mode = IntakeMode.FRONT;
		} else if (RobotContainer.intakeSubsystem.getBackSensor()) {
		RobotContainer.intakeSubsystem.setKickMotorSpeed(
			multiplier
				* -BBMath.getRps(
					IntakeConstants.intakeDps / IntakeConstants.kickGearRatio,
					IntakeConstants.kickWheelDiameter));
		this.mode = IntakeMode.BACK;
		} else {
		RobotContainer.intakeSubsystem.setKickMotorVoltage(0);
		}
		break;
	}
	// RobotContainer.intakeSubsystem.setGroundMotorVoltage(-12);
	// RobotContainer.intakeSubsystem.setGrabMotorVoltage(12);
	// RobotContainer.intakeSubsystem.setKickMotorVoltage(-12);
	/*if (RobotContainer.shooterSubsystem.getShooterSensor() == true) {
	end(true);
	}*/
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
	// RobotContainer.intakeSubsystem.setGrabMotorSpeed(0);
	// RobotContainer.intakeSubsystem.setGroundMotorSpeed(0);
	// RobotContainer.intakeSubsystem.setKickMotorSpeed(0);

	RobotContainer.intakeSubsystem.setChamberMotorVoltage(0);
	RobotContainer.intakeSubsystem.setGroundMotorVoltage(0);
	RobotContainer.intakeSubsystem.setKickMotorVoltage(0);
	RobotContainer.shooterSubsystem.setFeedMotorVoltage(0);
	mode = initialMode;
	intakeTimer.stop();
	intakeTimer.reset();
	timerStarted = false;
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
	if (RobotContainer.shooterSubsystem.getShooterSensor() && !timerStarted) {
	intakeTimer.start();
	timerStarted = true;
	}
	if (intakeTimer.advanceIfElapsed(0.05)) {
	return true;
	} else {
	return false;
	}
}
}
