// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class IntakeSubsystem extends SubsystemBase {
/** Creates a new IntakeSubsystem. */
private TalonFX groundMotor, kickMotor, grabMotor;

private DigitalInput frontSensor, backSensor;

public IntakeSubsystem() {
	groundMotor = new TalonFX(IDConstants.intakeMotorOne, "rio");
	kickMotor = new TalonFX(IDConstants.intakeMotorTwo, "rio");
	grabMotor = new TalonFX(IDConstants.intakeMotorThree, "rio");
	frontSensor = new DigitalInput(0);
	backSensor = new DigitalInput(0);
}

public void setGroundMotorSpeed(double rps) {
	groundMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setKickMotorSpeed(double rps) {
	kickMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setGrabMotorSpeed(double rps) {
	grabMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
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

public void log() {}

@Override
public void periodic() {
	// This method will be called once per scheduler run
}
}
