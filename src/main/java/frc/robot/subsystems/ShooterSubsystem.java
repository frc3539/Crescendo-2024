// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ShooterSubsystem extends SubsystemBase {
/** Creates a new ShooterSubsystem. */
private TalonFX topMotor, bottomMotor, feedMotor, angleMotor;

private DigitalInput shooterSensor;

public ShooterSubsystem() {
	topMotor = new TalonFX(IDConstants.topMotor, "rio");
	bottomMotor = new TalonFX(IDConstants.bottomMotor, "rio");
	feedMotor = new TalonFX(IDConstants.feedMotor, "rio");
	angleMotor = new TalonFX(0, "rio");

	shooterSensor = new DigitalInput(2);
}

public void setTopMotorSpeed(double rps) {
	topMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setTopMotorVoltage(double voltage) {
	topMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
}

public void setBottomMotorSpeed(double rps) {
	bottomMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setBottomMotorVoltage(double voltage) {
	bottomMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
}

public void setFeedMotorSpeed(double rps) {
	feedMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setFeedMotorVoltage(double voltage) {
	feedMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
}

public void setAngleMotorSpeed(double rps) {
	angleMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public boolean getShooterSensor() {
	if (shooterSensor.get() == true) {
	return true;
	} else {
	return false;
	}
}

public void log() {}

@Override
public void periodic() {
	// This method will be called once per scheduler run
}
}
