// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeCommand extends Command {
public enum IntakeMode {
	FRONT,
	BACK,
	SENSOR;
}

boolean intake;
IntakeMode mode;

// Creates a new IntakeCommand.
public IntakeCommand(boolean intake, IntakeMode mode) {
	addRequirements(RobotContainer.intakeSubsystem, RobotContainer.shooterSubsystem);
	this.intake = intake;
	this.mode = mode;
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
	if (RobotContainer.shooterSubsystem.getShooterSensor() && intake == true) {
	RobotContainer.intakeSubsystem.setGroundMotorVoltage(0);
	RobotContainer.intakeSubsystem.setGrabMotorVoltage(0);
	RobotContainer.intakeSubsystem.setKickMotorVoltage(0);
	RobotContainer.shooterSubsystem.setFeedMotorVoltage(0);

	return;
	}
	if (intake == true) {
	// RobotContainer.intakeSubsystem.setGroundMotorSpeed(IntakeConstants.intakeRps);
	// RobotContainer.intakeSubsystem.setGrabMotorSpeed(IntakeConstants.intakeRps);

	RobotContainer.intakeSubsystem.setGroundMotorVoltage(12);
	RobotContainer.intakeSubsystem.setGrabMotorVoltage(2);
	RobotContainer.shooterSubsystem.setFeedMotorVoltage(2);

	} else {
	// RobotContainer.intakeSubsystem.setGroundMotorSpeed(-IntakeConstants.intakeRps);
	// RobotContainer.intakeSubsystem.setGrabMotorSpeed(-IntakeConstants.intakeRps);

	RobotContainer.intakeSubsystem.setGroundMotorVoltage(-12);
	RobotContainer.intakeSubsystem.setGrabMotorVoltage(-12);
	RobotContainer.shooterSubsystem.setFeedMotorVoltage(-12);
	}

	switch (mode) {
	case FRONT:
		// RobotContainer.intakeSubsystem.setKickMotorSpeed(IntakeConstants.kickRps);
		RobotContainer.intakeSubsystem.setKickMotorVoltage(12);
		break;

	case BACK:
		// RobotContainer.intakeSubsystem.setKickMotorSpeed(-IntakeConstants.kickRps);
		RobotContainer.intakeSubsystem.setKickMotorVoltage(-12);

		break;

	case SENSOR:
		Boolean sensorReading = RobotContainer.intakeSubsystem.getSensor();
		if (sensorReading == true) {
		RobotContainer.intakeSubsystem.setKickMotorVoltage(12);
		}
		if (sensorReading == false) {
		RobotContainer.intakeSubsystem.setKickMotorVoltage(-12);
		}
		if (sensorReading == null) {
		RobotContainer.intakeSubsystem.setKickMotorVoltage(0);
		}
		break;
	}
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
	if (RobotContainer.shooterSubsystem.getShooterSensor() && intake == true) {
	RobotContainer.intakeSubsystem.setGroundMotorVoltage(0);
	RobotContainer.intakeSubsystem.setGrabMotorVoltage(0);
	RobotContainer.intakeSubsystem.setKickMotorVoltage(0);
	RobotContainer.shooterSubsystem.setFeedMotorVoltage(0);
	return;
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

	RobotContainer.intakeSubsystem.setGrabMotorVoltage(0);
	RobotContainer.intakeSubsystem.setGroundMotorVoltage(0);
	RobotContainer.intakeSubsystem.setKickMotorVoltage(0);
	RobotContainer.shooterSubsystem.setFeedMotorVoltage(0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
	return false;
}
}
