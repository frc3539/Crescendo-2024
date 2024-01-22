// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.DriveCommand;
import frc.robot.constants.DrivetrainConstants;
import java.util.Arrays;
import org.frcteam3539.Byte_Swerve_Lib.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.util.DrivetrainFeedforwardConstants;
import org.frcteam3539.Byte_Swerve_Lib.util.HolonomicFeedforward;

public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem {
  /** Creates a new DrivetrainSubsystem. */
  private final HolonomicMotionProfiledTrajectoryFollower follower;

  private SwerveRequest swerveRequest = new SwerveRequest.Idle();

  public double maxVelocity = 0.0;
  public double maxRotationalVelocity = 0.0;

  public DrivetrainSubsystem(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    maxVelocity = modules[0].SpeedAt12VoltsMps;

    Translation2d[] moduleLocations = new Translation2d[ModuleCount];

    for (int i = 0; i < modules.length; i++) {
      moduleLocations[i] = new Translation2d(modules[i].LocationX, modules[i].LocationY);
    }

    double dtRadius =
        new Translation2d()
            .nearest(Arrays.asList(moduleLocations))
            .getDistance(new Translation2d());
    maxRotationalVelocity = maxVelocity / dtRadius;

    DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS =
        new DrivetrainFeedforwardConstants(
            DrivetrainConstants.TranslationkV,
            DrivetrainConstants.TranslationkA,
            DrivetrainConstants.TranslationkS);

    follower =
        new HolonomicMotionProfiledTrajectoryFollower(
            new PidConstants(
                DrivetrainConstants.TranslationkP,
                DrivetrainConstants.TranslationkI,
                DrivetrainConstants.TranslationkD),
            new PidConstants(
                DrivetrainConstants.RotationkP,
                DrivetrainConstants.RotationkI,
                DrivetrainConstants.RotationkD),
            new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

    setDefaultCommand(new DriveCommand());
  }

  public void applyRequest(SwerveRequest request) {
    this.swerveRequest = request;
  }

  public void cancelPath() {
    follower.cancel();
  }

  public void followPath(Trajectory t) {
    follower.follow(t);
  }

  public void log() {}

  @Override
  public void periodic() {

    SwerveRequest request = new SwerveRequest.Idle();

    var driveSignalOpt =
        follower.update(
            this.m_odometry.getEstimatedPosition(),
            Timer.getFPGATimestamp(),
            Robot.defaultPeriodSecs);

    // If we should be running a profile use those chassisspeeds instead.
    if (driveSignalOpt.isPresent()) {
      ChassisSpeeds speeds = driveSignalOpt.get();
      request =
          new SwerveRequest.FieldCentric()
              .withVelocityX(speeds.vxMetersPerSecond)
              .withVelocityY(speeds.vyMetersPerSecond)
              .withRotationalRate(speeds.omegaRadiansPerSecond);
    } else request = swerveRequest;

    this.setControl(request);
    // This method will be called once per scheduler run
  }
}
