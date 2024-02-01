// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ClimberConstants;

public class BuddyClimbCommand extends Command {
/** Creates a new BuddyClimbCommand. */
public BuddyClimbCommand() {
	// Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
@Override
public void initialize() {

	RobotContainer.climberSubsystem.setBuddyClimbMotorSpeed(ClimberConstants.buddyClimbRps);
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
	if (RobotContainer.climberSubsystem.getBuddyClimbMotorSpeed()
		<= ClimberConstants.buddyClimbThreshold) {
	end(true);
	}
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
	RobotContainer.climberSubsystem.setBuddyClimbMotorSpeed(0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
	return false;
}
}
