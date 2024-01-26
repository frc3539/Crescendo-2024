// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
	private TalonFX leftClimbMotor, rightClimbMotor;

	public ClimberSubsystem() {
		leftClimbMotor = new TalonFX(IDConstants.leftClimbMotorID, "rio");
		rightClimbMotor = new TalonFX(IDConstants.rightClimbMotorID, "rio");

	}

	public void getLeftClimbMotorSpeed(double rps) {
		leftClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
	}

	public void getRightClimbMotorSpeed(double rps) {
		rightClimbMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));

	}

	public void log() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
