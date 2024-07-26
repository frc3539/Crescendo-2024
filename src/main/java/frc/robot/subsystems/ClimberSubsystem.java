// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
	private TalonFX leftClimbMotor, rightClimbMotor;
	StatusSignal<ForwardLimitValue> leftLimitSwitch, rightLimitSwitch;
	VelocityVoltage velocityVoltageControlLeft,
			velocityVoltageControlRight = new VelocityVoltage(0).withEnableFOC(true);
	VoltageOut voltageControlLeft, voltageControlRight = new VoltageOut(0).withEnableFOC(true);

	public ClimberSubsystem() {
		MotorOutputConfigs rightOutputConfig = new MotorOutputConfigs();
		rightOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
		// rightOutputConfig.Inverted = InvertedValue.Clockwise_Positive;

		MotorOutputConfigs leftOutputConfig = new MotorOutputConfigs();
		leftOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
		// leftOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

		MotorOutputConfigs buddyOutputConfig = new MotorOutputConfigs();
		buddyOutputConfig.Inverted = InvertedValue.CounterClockwise_Positive;

		leftClimbMotor = new TalonFX(IDConstants.leftClimbMotorID, "rio");
		leftClimbMotor.getConfigurator().apply(leftOutputConfig);
		rightClimbMotor = new TalonFX(IDConstants.rightClimbMotorID, "rio");
		rightClimbMotor.getConfigurator().apply(rightOutputConfig);

		leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
		rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);

		leftClimbMotor.getConfigurator().apply(new HardwareLimitSwitchConfigs().withForwardLimitEnable(true));
		rightClimbMotor.getConfigurator().apply(new HardwareLimitSwitchConfigs().withForwardLimitEnable(true));

		leftLimitSwitch = leftClimbMotor.getForwardLimit();
		rightLimitSwitch = rightClimbMotor.getForwardLimit();

	}

	public void setLeftClimbMotorSpeed(double rps) {
		leftClimbMotor.setControl(velocityVoltageControlLeft.withVelocity(rps));
	}

	public void setLeftClimbMotorVoltage(double voltage) {
		leftClimbMotor.setControl(voltageControlLeft.withOutput(voltage));
	}

	public void setRightClimbMotorSpeed(double rps) {
		rightClimbMotor.setControl(velocityVoltageControlRight.withVelocity(rps));
	}

	public void setRightClimbMotorVoltage(double voltage) {
		rightClimbMotor.setControl(voltageControlRight.withOutput(voltage));
	}

	public void setClimberBreakMode(boolean enabled) {
		if (enabled) {
			leftClimbMotor.setNeutralMode(NeutralModeValue.Brake);
			rightClimbMotor.setNeutralMode(NeutralModeValue.Brake);
		} else {
			leftClimbMotor.setNeutralMode(NeutralModeValue.Coast);
			rightClimbMotor.setNeutralMode(NeutralModeValue.Coast);
		}
	}

	public boolean doneClimbing() {
		return leftLimitSwitch.getValue() == ForwardLimitValue.ClosedToGround
				&& rightLimitSwitch.getValue() == ForwardLimitValue.ClosedToGround;

	}

	public void log() {
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
