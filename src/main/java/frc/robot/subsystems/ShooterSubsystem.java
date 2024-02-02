// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
/** Creates a new ShooterSubsystem. */
private TalonFX topMotor, bottomMotor, feedMotor, angleMotor;

private DigitalInput shooterSensor;

public ShooterSubsystem() {
	topMotor = new TalonFX(IDConstants.topMotor, "rio");
	bottomMotor = new TalonFX(IDConstants.bottomMotor, "rio");
	feedMotor = new TalonFX(IDConstants.feedMotor, "rio");
	angleMotor = new TalonFX(0, "rio");
	angleMotor
		.getConfigurator()
		.apply(
			new SoftwareLimitSwitchConfigs()
				.withForwardSoftLimitEnable(true)
				.withForwardSoftLimitThreshold(ShooterConstants.angleShooterSoftMax)
				.withReverseSoftLimitEnable(true)
				.withReverseSoftLimitThreshold(ShooterConstants.angleShooterSoftMin));
	angleMotor
		.getConfigurator()
		.apply(
			new SlotConfigs()
				.withKP(ShooterConstants.angleShooterP)
				.withKI(ShooterConstants.angleShooterI)
				.withKD(ShooterConstants.angleShooterD)
				.withKV(ShooterConstants.angleShooterV)
				.withKG(ShooterConstants.angleShooterG)
				.withGravityType(GravityTypeValue.Arm_Cosine));
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

public void setShooterAngle(int angle) {
	// angleMotor.setControl
}

public boolean getShooterSensor() {
	if (shooterSensor.get() == true) {
	return true;
	} else {
	return false;
	}
}

public double getTopMotorSpeed() {
	return topMotor.getVelocity().getValue();
}

public double getBottomMotorSpeed() {
	return bottomMotor.getVelocity().getValue();
}

public void log() {}

@Override
public void periodic() {
	// This method will be called once per scheduler run
}
}
