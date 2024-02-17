// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveCommand extends Command {
/** Creates a new DriveCommand. */
double maxVelocity = RobotContainer.drivetrainSubsystem.maxVelocity;

double maxRotationalVelocity = RobotContainer.drivetrainSubsystem.maxRotationalVelocity;

private final SwerveRequest.FieldCentric driveFieldCentric =
	new SwerveRequest.FieldCentric()
		.withDeadband(maxVelocity * 0.1)
		.withRotationalDeadband(maxRotationalVelocity * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

private final SwerveRequest.RobotCentric driveRobotCentric =
	new SwerveRequest.RobotCentric()
		.withDeadband(maxVelocity * 0.1)
		.withRotationalDeadband(maxRotationalVelocity * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

public DriveCommand() {
	addRequirements(RobotContainer.drivetrainSubsystem);
}

// Called when the command is initially scheduled.
@Override
public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {

	SwerveRequest request = new SwerveRequest.Idle();

	request =
		driveFieldCentric
			.withVelocityX(-RobotContainer.driverController.getLeftY() * maxVelocity)
			.withVelocityY(-RobotContainer.driverController.getLeftX() * maxVelocity)
			.withRotationalRate(
				-RobotContainer.driverController.getRightX() * maxRotationalVelocity);

	if (RobotContainer.driverController.rightTrigger(0.5).getAsBoolean()) {
	request =
		driveRobotCentric
			.withVelocityX(-RobotContainer.driverController.getLeftY() * maxVelocity)
			.withVelocityY(-RobotContainer.driverController.getLeftX() * maxVelocity)
			.withRotationalRate(
				-RobotContainer.driverController.getRightX() * maxRotationalVelocity);
	}

	RobotContainer.drivetrainSubsystem.applyRequest(request);
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {}

// Returns true when the command should end.
@Override
public boolean isFinished() {
	return false;
}

private static Translation2d modifyJoystick(Translation2d joystick) {
	// Deadband
	Rotation2d rotation = joystick.getAngle();
	double distance = joystick.getNorm();

	double distanceModified = Math.copySign(Math.pow(distance, 3), distance);

	return new Translation2d(distanceModified, rotation);
}
}
