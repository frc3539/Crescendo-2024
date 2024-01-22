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
  private final SwerveRequest.FieldCentric driveFieldCentric =
      new SwerveRequest.FieldCentric()
          .withDeadband(RobotContainer.drivetrainSubsystem.maxVelocity * 0.05)
          .withRotationalDeadband(
              RobotContainer.drivetrainSubsystem.maxRotationalVelocity * 0.05) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  // driving in open loop

  public DriveCommand() {
    addRequirements(RobotContainer.drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.drivetrainSubsystem.applyRequest(
        driveFieldCentric.withVelocityX(RobotContainer.driver.getLeftY()));
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
