// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RevUpCommand extends Command {
/** Creates a new RevUpCommand. */
public RevUpCommand() {
	// Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
	// RobotContainer.shooterSubsystem.setBottomMotorSpeed(ShooterConstants.revRps);
	// RobotContainer.shooterSubsystem.setTopMotorSpeed(-ShooterConstants.revRps);

}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
	RobotContainer.shooterSubsystem.setBottomMotorVoltage(12);
	RobotContainer.shooterSubsystem.setTopMotorVoltage(12);
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
	// RobotContainer.shooterSubsystem.setBottomMotorSpeed(0);
	// RobotContainer.shooterSubsystem.setTopMotorSpeed(0);
	System.out.println("END");
	RobotContainer.shooterSubsystem.setBottomMotorVoltage(0);
	RobotContainer.shooterSubsystem.setTopMotorVoltage(0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
	return false;
}
}
