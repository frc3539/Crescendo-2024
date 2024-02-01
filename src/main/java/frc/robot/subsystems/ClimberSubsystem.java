// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
private TalonFX leftClimbMotor, rightClimbMotor, buddyClimbMotor;

public ClimberSubsystem() {
	leftClimbMotor = new TalonFX(IDConstants.leftClimbMotorID, "rio");
	rightClimbMotor = new TalonFX(IDConstants.rightClimbMotorID, "rio");
	buddyClimbMotor = new TalonFX(IDConstants.buddyClimbMotorID, "rio");
}

public void setLeftClimbMotorSpeed(double rps) {
	leftClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setRightClimbMotorSpeed(double rps) {
	rightClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setBuddyClimbMotorSpeed(double rps) {
	buddyClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
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
