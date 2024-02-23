// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
private TalonFX leftClimbMotor, rightClimbMotor, buddyClimbMotor;

public ClimberSubsystem() {
	MotorOutputConfigs rightOutputConfig = new MotorOutputConfigs();
	rightOutputConfig.Inverted = InvertedValue.Clockwise_Positive;

	MotorOutputConfigs leftOutputConfig = new MotorOutputConfigs();
	leftOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

	MotorOutputConfigs buddyOutputConfig = new MotorOutputConfigs();
	buddyOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

	leftClimbMotor = new TalonFX(IDConstants.leftClimbMotorID, "rio");
	leftClimbMotor.getConfigurator().apply(leftOutputConfig);
	rightClimbMotor = new TalonFX(IDConstants.rightClimbMotorID, "rio");
	rightClimbMotor.getConfigurator().apply(rightOutputConfig);
	buddyClimbMotor = new TalonFX(IDConstants.buddyClimbMotorID, "rio");
	buddyClimbMotor.getConfigurator().apply(buddyOutputConfig);

	leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
	rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);
}

public void setLeftClimbMotorSpeed(double rps) {
	leftClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setLeftClimbMotorVoltage(double voltage) {
	leftClimbMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
}

public void setRightClimbMotorSpeed(double rps) {
	rightClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setRightClimbMotorVoltage(double voltage) {
	rightClimbMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
}

public void setBuddyClimbMotorSpeed(double rps) {
	buddyClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setBuddyClimbMotorVoltage(double voltage) {
	buddyClimbMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
}

public double getBuddyClimbMotorSpeed() {

	return buddyClimbMotor.getVelocity().getValue();
}

public void log() {}

@Override
public void periodic() {
	// This method will be called once per scheduler run
}
}
