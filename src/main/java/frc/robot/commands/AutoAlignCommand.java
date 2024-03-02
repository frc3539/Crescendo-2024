// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;
import org.frcteam3539.Byte_Swerve_Lib.control.MaxAccelerationConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.MaxVelocityConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.SimplePathBuilder;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.control.TrajectoryConstraint;

public class AutoAlignCommand extends Command {
/** Wrapper command to generate a trajectory to the nearest Stage Pose */
public enum TagPosition {
	TRAP,
	CLIMB,
	SPEAKER,
	AMP,
	SOURCE
}

public List<Pose2d> points = new ArrayList<Pose2d>();

public AutoAlignCommand(TagPosition position) {

	switch (position) {
	case AMP:
		points.add(new Pose2d(1, 1, Rotation2d.fromDegrees(-60)));
		break;
	case SPEAKER:
		points.add(new Pose2d(1, 1, Rotation2d.fromDegrees(-60)));
		break;
	case TRAP:
		points.add(new Pose2d(4.961, 4.216, Rotation2d.fromDegrees(120)));
		break;
	case CLIMB:
		points.add(new Pose2d(1, 1, Rotation2d.fromDegrees(-60)));
		break;
	case SOURCE:
		points.add(new Pose2d(1, 1, Rotation2d.fromDegrees(-60)));
		break;
	}
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
	Pose2d robotPose = RobotContainer.drivetrainSubsystem.getPose2d();

	Pose2d target = robotPose.nearest(points);

	// Generate trajectory command to nearest coordinate
	RobotContainer.drivetrainSubsystem
		.getFollower()
		.follow(
			new Trajectory(
				new SimplePathBuilder(robotPose).lineTo(target).build(),
				new TrajectoryConstraint[] {
				(TrajectoryConstraint) new MaxAccelerationConstraint(1),
				(TrajectoryConstraint) new MaxVelocityConstraint(1)
				},
				.05));
}

// Indicate vision and start the trajectory command

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
	RobotContainer.drivetrainSubsystem.getFollower().cancel();
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
	return RobotContainer.drivetrainSubsystem.getFollower().getCurrentTrajectory().isEmpty();
}
}
