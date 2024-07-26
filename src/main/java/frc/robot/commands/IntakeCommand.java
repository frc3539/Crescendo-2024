// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.BBMath;

public class IntakeCommand extends Command {
	public enum IntakeMode {
		FRONT, BACK, SENSOR;
	}

	boolean intaking;
	IntakeMode mode;
	IntakeMode initialMode;
	Timer intakeTimer = new Timer();
	boolean timerStarted = false;

	// Creates a new IntakeCommand.
	public IntakeCommand(boolean intaking, IntakeMode mode) {
		this.intaking = intaking;
		this.mode = mode;
		this.initialMode = mode;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		LedSubsystem.setIntaking(true);

		if (ShooterSubsystem.getShooterSensor() && intaking == true) {
			IntakeSubsystem.setGroundMotorVoltage(0);
			IntakeSubsystem.setChamberMotorVoltage(0);
			IntakeSubsystem.setKickMotorVoltage(0);
			ShooterSubsystem.setFeedMotorVoltage(0);

			return;
		}
		if (intaking == true) {
			// IntakeSubsystem.setGroundMotorSpeed(IntakeConstants.intakeRps);
			// IntakeSubsystem.setGrabMotorSpeed(IntakeConstants.intakeRps);

			IntakeSubsystem.setGroundMotorSpeed(
					BBMath.getRps(IntakeConstants.intakeDps * 3, IntakeConstants.groundWheelDiameter));
			IntakeSubsystem.setChamberMotorSpeed(
					BBMath.getRps(IntakeConstants.intakeDps, IntakeConstants.chamberWheelDiameter));
			ShooterSubsystem
					.setFeedMotorSpeed(BBMath.getRps(IntakeConstants.intakeDps, ShooterConstants.feedWheelDiameter));

		} else {
			// IntakeSubsystem.setGroundMotorSpeed(-IntakeConstants.intakeRps);
			// IntakeSubsystem.setGrabMotorSpeed(-IntakeConstants.intakeRps);

			ShooterSubsystem.setFeedMotorSpeed(-IntakeConstants.intakeDps);
			ShooterSubsystem.setTopMotorSpeed(-IntakeConstants.intakeDps);
			ShooterSubsystem.setBottomMotorSpeed(-IntakeConstants.intakeDps);

			IntakeSubsystem.setGroundMotorSpeed(
					BBMath.getRps(-IntakeConstants.intakeDps * 3, IntakeConstants.groundWheelDiameter));
			IntakeSubsystem.setChamberMotorSpeed(
					BBMath.getRps(-IntakeConstants.intakeDps, IntakeConstants.chamberWheelDiameter));
			ShooterSubsystem
					.setFeedMotorSpeed(BBMath.getRps(-IntakeConstants.intakeDps, ShooterConstants.feedWheelDiameter));
			IntakeSubsystem.setKickMotorSpeed(BBMath.getRps(IntakeConstants.intakeDps / IntakeConstants.kickGearRatio,
					IntakeConstants.kickWheelDiameter));
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (!intaking) {
			return;
		}
		// if (ShooterSubsystem.getShooterSensor() && intake == true) {
		// IntakeSubsystem.setGroundMotorVoltage(0);
		// IntakeSubsystem.setChamberMotorVoltage(0);
		// IntakeSubsystem.setKickMotorVoltage(0);
		// ShooterSubsystem.setFeedMotorVoltage(0);
		// return;
		// }

		double multiplier = 1;
		if (IntakeSubsystem.getChamberSensor()) {
			multiplier = 0.5;
			// IntakeSubsystem.setGroundMotorVoltage(0);
		}

		if (IntakeSubsystem.getChamberSensor() && intaking == true) {
			IntakeSubsystem.setChamberMotorSpeed(
					multiplier * BBMath.getRps(IntakeConstants.intakeDps, IntakeConstants.chamberWheelDiameter));
			ShooterSubsystem.setFeedMotorSpeed(
					multiplier * BBMath.getRps(IntakeConstants.intakeDps, ShooterConstants.feedWheelDiameter));
		}

		switch (mode) {
			case FRONT :
				// IntakeSubsystem.setKickMotorSpeed(IntakeConstants.kickRps);
				IntakeSubsystem.setKickMotorSpeed(multiplier * BBMath.getRps(
						IntakeConstants.intakeDps / IntakeConstants.kickGearRatio, IntakeConstants.kickWheelDiameter));
				break;

			case BACK :
				// IntakeSubsystem.setKickMotorSpeed(-IntakeConstants.kickRps);
				IntakeSubsystem.setKickMotorSpeed(
						multiplier * -BBMath.getRps(IntakeConstants.intakeDps / IntakeConstants.kickGearRatio,
								IntakeConstants.kickWheelDiameter));
				break;

			case SENSOR :
				if (IntakeSubsystem.getFrontSensor()) {
					IntakeSubsystem.setKickMotorSpeed(
							multiplier * BBMath.getRps(IntakeConstants.intakeDps / IntakeConstants.kickGearRatio,
									IntakeConstants.kickWheelDiameter));
					this.mode = IntakeMode.FRONT;
				} else if (IntakeSubsystem.getBackSensor()) {
					IntakeSubsystem.setKickMotorSpeed(
							multiplier * -BBMath.getRps(IntakeConstants.intakeDps / IntakeConstants.kickGearRatio,
									IntakeConstants.kickWheelDiameter));
					this.mode = IntakeMode.BACK;
				} else {
					IntakeSubsystem.setKickMotorVoltage(0);
				}
				break;
		}
		// IntakeSubsystem.setGroundMotorVoltage(-12);
		// IntakeSubsystem.setGrabMotorVoltage(12);
		// IntakeSubsystem.setKickMotorVoltage(-12);
		/*
		 * if (ShooterSubsystem.getShooterSensor() == true) { end(true); }
		 */
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// IntakeSubsystem.setGrabMotorSpeed(0);
		// IntakeSubsystem.setGroundMotorSpeed(0);
		// IntakeSubsystem.setKickMotorSpeed(0);
		if (!intaking) {
			ShooterSubsystem.setFeedMotorSpeed(0);
			ShooterSubsystem.setTopMotorSpeed(0);
			ShooterSubsystem.setBottomMotorSpeed(0);
		}
		LedSubsystem.setIntaking(false);

		IntakeSubsystem.setChamberMotorVoltage(0);
		IntakeSubsystem.setGroundMotorVoltage(0);
		IntakeSubsystem.setKickMotorVoltage(0);
		ShooterSubsystem.setFeedMotorVoltage(0);
		mode = initialMode;
		intakeTimer.stop();
		intakeTimer.reset();
		timerStarted = false;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (!intaking) {
			return false;
		}
		if (ShooterSubsystem.getShooterSensor() && !timerStarted) {
			intakeTimer.start();
			timerStarted = true;
		}
		if (intakeTimer.advanceIfElapsed(IntakeConstants.intakeShutOffDelay)) {
			return true;
		} else {
			return false;
		}
	}
}
