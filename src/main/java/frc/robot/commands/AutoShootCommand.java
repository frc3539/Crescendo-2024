// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoShootCommand extends Command {

	double targetZOffset = -0.0254;

	Translation2d blueSpeakerCoordinate = new Translation2d(0, 5.55);
	Translation2d redSpeakerCoordinate = new Translation2d(0, 2.67);

	/** Creates a new AutoShootCommand. */
	public AutoShootCommand() {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// RobotContainer.shooterSubsystem
		// .setTopMotorSpeed(BBMath.getRps(ShooterConstants.shootDps,
		// ShooterConstants.shootWheelDiameter));
		// RobotContainer.shooterSubsystem
		// .setBottomMotorSpeed(BBMath.getRps(ShooterConstants.shootDps,
		// ShooterConstants.shootWheelDiameter));
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// if (RobotContainer.shooterSubsystem.getShooterSensor() == false) {
		// RobotContainer.shooterSubsystem.setFeedMotorSpeed(0);
		// }
		// if (!MathUtil.isNear(BBMath.getRps(ShooterConstants.shootDps,
		// ShooterConstants.shootWheelDiameter),
		// RobotContainer.shooterSubsystem.getTopMotorSpeed(), 5)) {
		// return;
		// }
		// if (!MathUtil.isNear(BBMath.getRps(ShooterConstants.shootDps,
		// ShooterConstants.shootWheelDiameter),
		// RobotContainer.shooterSubsystem.getBottomMotorSpeed(), 5)) {
		// return;
		// }
		// RobotContainer.shooterSubsystem
		// .setFeedMotorSpeed(BBMath.getRps(ShooterConstants.feedDps,
		// ShooterConstants.feedWheelDiameter));
		double distanceToTarget = RobotContainer.drivetrainSubsystem.getPose2d().getTranslation()
				.getDistance(blueSpeakerCoordinate);
		org.littletonrobotics.junction.Logger.recordOutput("/Drivetrain/DistanceToTarget", distanceToTarget);
		// double angleToTarget = (Math.atan2(0.64135 - 2.0452 + targetZOffset,
		// distanceToTarget - 0.3048 - 0.2286)
		// + Math.toRadians(0 - Math.min(Math.max(0, distanceToTarget - 0.3048 - 1) * 1,
		// 10))) * 180 / Math.PI;

		double angleToTarget = -101 + 42.6667 * distanceToTarget - 8.25 * Math.pow(distanceToTarget, 2)
				+ 0.5833 * Math.pow(distanceToTarget, 3);
		angleToTarget = Math.max(angleToTarget, -55);

		RobotContainer.shooterSubsystem.setShooterAngle(angleToTarget);
		org.littletonrobotics.junction.Logger.recordOutput("/Drivetrain/EstimatedAngle", angleToTarget);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		RobotContainer.shooterSubsystem.setTopMotorVoltage(0);
		RobotContainer.shooterSubsystem.setBottomMotorVoltage(0);
		RobotContainer.shooterSubsystem.setFeedMotorVoltage(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
