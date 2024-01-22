// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private TalonFX topMotor, bottomMotor, feedMotor;

  public ShooterSubsystem() {
    topMotor = new TalonFX(IDConstants.topMotor, "canivore");
    bottomMotor = new TalonFX(IDConstants.bottomMotor, "canivore");
    feedMotor = new TalonFX(IDConstants.feedMotor, "canivore");
  }

  public void setTopMotorSpeed(double rps) {
    topMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
  }

  public void setBottomMotorSpeed(double rps) {
    bottomMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
  }

  public void setFeedMotorSpeed(double rps) {
    feedMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
  }

  public void log() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
