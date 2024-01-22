// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class IntakeSubsystem extends SubsystemBase {
/** Creates a new IntakeSubsystem. */
private TalonFX intakeMotorOne, intakeMotorTwo, intakeMotorThree;

public IntakeSubsystem() {
	intakeMotorOne = new TalonFX(IDConstants.intakeMotorOne, "rio");
	intakeMotorTwo = new TalonFX(IDConstants.intakeMotorTwo, "rio");
	intakeMotorThree = new TalonFX(IDConstants.intakeMotorThree, "rio");
}

public void setMotorOneSpeed(double rps) {
	intakeMotorOne.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setMotorTwoSpeed(double rps) {
	intakeMotorTwo.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void setMotorThreeSpeed(double rps) {
	intakeMotorThree.setControl(new VelocityVoltage(rps).withEnableFOC(true));
}

public void log() {}

@Override
public void periodic() {
	// This method will be called once per scheduler run
}
}
