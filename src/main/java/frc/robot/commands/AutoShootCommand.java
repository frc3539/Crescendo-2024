// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ShooterConstants;

public class AutoShootCommand extends Command {
/** Creates a new AutoShootCommand. */
public AutoShootCommand() {
	// Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
	RobotContainer.shooterSubsystem.setTopMotorSpeed(ShooterConstants.revRps);
	RobotContainer.shooterSubsystem.setBottomMotorSpeed(-ShooterConstants.revRps);
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
	if (RobotContainer.shooterSubsystem.getShooterSensor() == false) {
	RobotContainer.shooterSubsystem.setFeedMotorSpeed(0);
	}
	if (RobotContainer.shooterSubsystem.getTopMotorSpeed() - ShooterConstants.revRps >= 1) {
	return;
	}
	if (RobotContainer.shooterSubsystem.getBottomMotorSpeed() + ShooterConstants.revRps >= 1) {
	return;
	}
	RobotContainer.shooterSubsystem.setFeedMotorSpeed(ShooterConstants.shootRps);
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
	RobotContainer.shooterSubsystem.setTopMotorSpeed(0);
	RobotContainer.shooterSubsystem.setBottomMotorSpeed(0);
	RobotContainer.shooterSubsystem.setFeedMotorSpeed(0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
	return false;
}
}
