// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import org.frcteam3539.Byte_Swerve_Lib.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.util.DrivetrainFeedforwardConstants;
import org.frcteam3539.Byte_Swerve_Lib.util.HolonomicFeedforward;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem {
  /** Creates a new DrivetrainSubsystem. */

  private final HolonomicMotionProfiledTrajectoryFollower follower;

  public DrivetrainSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {

    super(null);

    DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
        DrivetrainConstants.TranslationkV,
        DrivetrainConstants.TranslationkA, DrivetrainConstants.TranslationkS);

    follower = new HolonomicMotionProfiledTrajectoryFollower(
        new PidConstants(DrivetrainConstants.TranslationkP, DrivetrainConstants.TranslationkI,
            DrivetrainConstants.TranslationkD),
        new PidConstants(DrivetrainConstants.RotationkP, DrivetrainConstants.RotationkI,
            DrivetrainConstants.RotationkD),
        new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void log() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
