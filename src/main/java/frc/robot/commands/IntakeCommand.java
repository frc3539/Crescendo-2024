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

	int intakeSpeed = -1;
	int extakeSpeed = 1;

	int frontSpeed = -1;
	int backSpeed = 1;

	/** Creates a new IntakeCommand. */
	public IntakeCommand(boolean intake, IntakeMode mode) {
		this.intake = intake;
		this.mode = mode;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

		if (intake == true) {
			RobotContainer.intakeSubsystem.setGroundMotorSpeed(intakeSpeed);
			RobotContainer.intakeSubsystem.setGrabMotorSpeed(intakeSpeed);
		} else {
			RobotContainer.intakeSubsystem.setGroundMotorSpeed(extakeSpeed);
			RobotContainer.intakeSubsystem.setGrabMotorSpeed(extakeSpeed);
		}

		switch (mode) {
			case FRONT:
				RobotContainer.intakeSubsystem.setKickMotorSpeed(frontSpeed);
				break;

			case BACK:
				RobotContainer.intakeSubsystem.setKickMotorSpeed(backSpeed);
				break;

			case SENSOR:
				Boolean sensorReading = RobotContainer.intakeSubsystem.getSensor();
				if (sensorReading == true) {
					RobotContainer.intakeSubsystem.setKickMotorSpeed(frontSpeed);
				}
				if (sensorReading == false) {
					RobotContainer.intakeSubsystem.setKickMotorSpeed(backSpeed);
				}
				if (sensorReading == null) {
					RobotContainer.intakeSubsystem.setKickMotorSpeed(0);
				}
				break;
		}

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	RobotContainer.intakeSubsystem.setGrabMotorSpeed(0);
	RobotContainer.intakeSubsystem.setGroundMotorSpeed(0);
	RobotContainer.intakeSubsystem.setKickMotorSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
