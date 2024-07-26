// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utilities.BBMath;

public class RevUpCommand extends Command {

	private boolean endOnShoot;
	private double shootSpeed;
	private boolean timerStarted = false;
	private Timer shootTimer = new Timer();

	/** Creates a new RevUpCommand. */
	public RevUpCommand(boolean endOnShoot, double shootSpeed) {
		this.endOnShoot = endOnShoot;
		this.shootSpeed = shootSpeed;

		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// ShooterSubsystem.setBottomMotorSpeed(ShooterConstants.revRps);
		// ShooterSubsystem.setTopMotorSpeed(-ShooterConstants.revRps);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		ShooterSubsystem.setBottomMotorSpeed(BBMath.getRps(shootSpeed, ShooterConstants.shootWheelDiameter));
		ShooterSubsystem.setTopMotorSpeed(BBMath.getRps(shootSpeed, ShooterConstants.shootWheelDiameter));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// ShooterSubsystem.setBottomMotorSpeed(0);
		// ShooterSubsystem.setTopMotorSpeed(0);
		ShooterSubsystem.setBottomMotorVoltage(0);
		ShooterSubsystem.setTopMotorVoltage(0);
		shootTimer.stop();
		shootTimer.reset();
		timerStarted = false;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (!endOnShoot) {
			return false;
		}
		if (!ShooterSubsystem.getShooterSensor() && !timerStarted) {
			shootTimer.start();
			timerStarted = true;
		}
		if (shootTimer.advanceIfElapsed(1)) {
			return true;
		} else {
			return false;
		}
	}
}
