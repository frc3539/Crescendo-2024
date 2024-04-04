// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.DrivetrainConstants;

public class AutonNoteTrackCommand extends Command {
	/** Creates a new AutonNoteTrackCommand. */
	public AutonNoteTrackCommand() {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
SwerveRequest request = new SwerveRequest.Idle();

		double speedMultiplier = DrivetrainConstants.speedMultiplier;
		double rotationSpeedMultiplier = DrivetrainConstants.rotationSpeedMultiplier;

    var target = RobotContainer.visionSubsystem.getBestFrontNote();
			if (target != null & !RobotContainer.intakeSubsystem.getBackSensor()
					& !RobotContainer.intakeSubsystem.getFrontSensor()
					& !RobotContainer.intakeSubsystem.getChamberSensor()
					& !RobotContainer.shooterSubsystem.getShooterSensor()) {
				RobotContainer.ledSubsystem.setNoteTracking(true);

				double noteTrackSpeedMultiplier = 0.3;
				var angleToTarget = -target.getYaw() * Math.PI / 180;
				rotationController.setSetpoint(
						RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians() + angleToTarget);
				request = driveRobotCentric.withVelocityX(maxVelocity * noteTrackSpeedMultiplier).withVelocityY(0);
				driveRobotCentric.withRotationalRate(rotationController
						.calculate(RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians(), 0.02)
						* maxRotationalVelocity * .3).withRotationalDeadband(0);
				driveFieldCentric.withRotationalRate(rotationController
						.calculate(RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians(), 0.02)
						* maxRotationalVelocity * .3).withRotationalDeadband(0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
