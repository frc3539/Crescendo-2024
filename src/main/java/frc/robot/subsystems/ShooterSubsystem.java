// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import frc.robot.constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
/** Creates a new ShooterSubsystem. */
private TalonFX topMotor, bottomMotor, feedMotor, elevatorMotor, angleMotor;

private CANcoder angleCanCoder;

private DigitalInput shooterSensor;

public ShooterSubsystem() {

	angleCanCoder = new CANcoder(IDConstants.angleCanCoderID, "rio");
	topMotor = new TalonFX(IDConstants.topMotor, "rio");
	topMotor.getConfigurator().apply(new TalonFXConfiguration());
	topMotor.setInverted(true);
	bottomMotor = new TalonFX(IDConstants.bottomMotor, "rio");
	bottomMotor.getConfigurator().apply(new TalonFXConfiguration());
	bottomMotor.setInverted(true);
	feedMotor = new TalonFX(IDConstants.feedMotor, "rio");
	feedMotor.getConfigurator().apply(new TalonFXConfiguration());
	feedMotor.setInverted(true);
	feedMotor.setNeutralMode(NeutralModeValue.Brake);
	elevatorMotor = new TalonFX(IDConstants.elevatorMotorID, "rio");
	angleMotor = new TalonFX(IDConstants.angleMotorID, "rio");

	reloadFromConfig();

	elevatorMotor
		.getConfigurator()
		.apply(
			new SoftwareLimitSwitchConfigs()
				.withForwardSoftLimitEnable(true)
				.withForwardSoftLimitThreshold(ShooterConstants.elevatorSoftMax)
				.withReverseSoftLimitEnable(true)
				.withReverseSoftLimitThreshold(ShooterConstants.elevatorSoftMin));

	elevatorMotor
		.getConfigurator()
		.apply(
			new HardwareLimitSwitchConfigs()
				.withReverseLimitEnable(true)
				.withReverseLimitAutosetPositionEnable(true)
				.withReverseLimitAutosetPositionValue(0));

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
			new FeedbackConfigs()
				.withFeedbackRemoteSensorID(IDConstants.angleCanCoderID)
				.withRotorToSensorRatio(ShooterConstants.angleMotorToEncoder)
				.withSensorToMechanismRatio(1)
				.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));

	shooterSensor = new DigitalInput(IDConstants.shooterSensorChannel);
}

public void reloadFromConfig() {

	angleCanCoder
		.getConfigurator()
		.apply(
			new MagnetSensorConfigs()
				.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
				.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
				.withMagnetOffset(ShooterConstants.shooterAngleOffset));
	elevatorMotor
		.getConfigurator()
		.apply(
			new SlotConfigs()
				.withKP(ShooterConstants.angleShooterP)
				.withKI(ShooterConstants.elevatorMotorI)
				.withKD(ShooterConstants.elevatorMotorD)
				.withKV(ShooterConstants.elevatorMotorV)
				.withKG(ShooterConstants.elevatorMotorG)
				.withGravityType(GravityTypeValue.Elevator_Static));

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
}

public double getShooterAngle() {
	return angleCanCoder.getAbsolutePosition().getValue();
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

public void setElevatorMotorSpeed(double rps) {
	elevatorMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setAngleMotorSpeed(double rps) {
	angleMotor.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setShooterAngle(double angle) {
	angleMotor.setControl(new MotionMagicVoltage(Units.degreesToRotations(angle)));
}

public boolean getShooterSensor() {
	return !shooterSensor.get();
}

public double getTopMotorSpeed() {
	return topMotor.getVelocity().getValue();
}

public double getBottomMotorSpeed() {
	return bottomMotor.getVelocity().getValue();
}

public void log() {
	Logger.recordOutput("/Shooter/Shooter_Sensor", getShooterSensor());
}

@Override
public void periodic() {
	// This method will be called once per scheduler run
}
}
