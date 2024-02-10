// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class IntakeSubsystem extends SubsystemBase {
/** Creates a new IntakeSubsystem. */
private TalonFX groundMotor, kickMotor, grabMotor;

private DigitalInput frontSensor, backSensor, chamberSensor;

public IntakeSubsystem() {
	groundMotor = new TalonFX(IDConstants.groundMotorID, "rio");
	groundMotor.getConfigurator().apply(new TalonFXConfiguration());
	groundMotor.setInverted(true);
	kickMotor = new TalonFX(IDConstants.kickMotorID, "rio");
	kickMotor.getConfigurator().apply(new TalonFXConfiguration());
	kickMotor.setInverted(true);
	grabMotor = new TalonFX(IDConstants.grabMotorID, "rio");
	frontSensor = new DigitalInput(IDConstants.frontSensorChannel);
	backSensor = new DigitalInput(IDConstants.backSensorChannel);
	chamberSensor = new DigitalInput(IDConstants.chamberSensorChannel);
}

public void setGroundMotorSpeed(double rps) {
	groundMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setGroundMotorVoltage(double voltage) {
	groundMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
}

public void setKickMotorSpeed(double rps) {
	kickMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setKickMotorVoltage(double voltage) {
	kickMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
}

public void setGrabMotorSpeed(double rps) {
	grabMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setGrabMotorVoltage(double voltage) {
	grabMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
}

public Boolean getSensor() {
	if (frontSensor.get() == true) {
	return true;
	}
	if (backSensor.get() == true) {
	return false;
	}
	return null;
}

public boolean getChamberSensor() {
	if (chamberSensor.get() == true) {
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
