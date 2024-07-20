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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
	private TalonFX leftClimbMotor, rightClimbMotor;	
	// Simulation classes help us simulate what's going on, including gravity.
  	private ElevatorSim elevatorSimLeft, elevatorSimRight;
  


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


		if(RobotBase.isSimulation())
		{
			elevatorSimLeft =
				new ElevatorSim(
					DCMotor.getKrakenX60Foc(1),
					25,//25:1
					Units.lbsToKilograms(-120)/2.0, //Half the robot weight per side
					Units.inchesToMeters(1),
					0,
					Units.inchesToMeters(60),
					false,
					0,
					VecBuilder.fill(0.01));

			elevatorSimRight =
				new ElevatorSim(
					DCMotor.getKrakenX60Foc(1),
					25,//25:1
					Units.lbsToKilograms(-120)/2.0, //Half the robot weight per side
					Units.inchesToMeters(1),
					0,
					Units.inchesToMeters(60),
					false,
					0,
					VecBuilder.fill(0.01));
		}
		
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
		return leftClimbMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround
				&& rightClimbMotor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;

	}

	public void log() {
	}

	StatusSignal<Double> leftMotorCurrent = leftClimbMotor.getSupplyCurrent();
	StatusSignal<Double> rightMotorCurrent = rightClimbMotor.getSupplyCurrent();
	public double getTotalCurrent()
	{
		if(RobotBase.isSimulation())
		{
			return elevatorSimLeft.getCurrentDrawAmps() + elevatorSimRight.getCurrentDrawAmps();
		}
		return leftMotorCurrent.getValueAsDouble() + rightMotorCurrent.getValueAsDouble();
	}

	@Override
	public void periodic() {
		if(RobotBase.isSimulation())
		{
			var leftClimbMotorSim = leftClimbMotor.getSimState();
			var rightClimbMotorSim = rightClimbMotor.getSimState();

			leftClimbMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
			rightClimbMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

			// get the motor voltage of the TalonFX
			var leftClimbMotorVoltage = leftClimbMotorSim.getMotorVoltage();
			var rightClimbMotorVoltage = rightClimbMotorSim.getMotorVoltage();

			elevatorSimLeft.setInputVoltage(leftClimbMotorVoltage);
			elevatorSimLeft.update(0.020);

			elevatorSimRight.setInputVoltage(rightClimbMotorVoltage);
			elevatorSimRight.update(0.020);

			// apply the new rotor position and velocity to the TalonFX;
			// note that this is rotor position/velocity (before gear ratios)
			leftClimbMotorSim.setRawRotorPosition(Units.metersToInches((elevatorSimLeft.getPositionMeters())/(1.0*Math.PI))*25); // Account for the diameter of the drum, and the gear ratio.
			leftClimbMotorSim.setRotorVelocity(Units.metersToInches((elevatorSimLeft.getVelocityMetersPerSecond())/(1.0*Math.PI))*25);

			// apply the new rotor position and velocity to the TalonFX;
			// note that this is rotor position/velocity (before gear ratios)
			rightClimbMotorSim.setRawRotorPosition(Units.metersToInches((elevatorSimRight.getPositionMeters())/(1.0*Math.PI))*25); // Account for the diameter of the drum, and the gear ratio.
			rightClimbMotorSim.setRotorVelocity(Units.metersToInches((elevatorSimRight.getVelocityMetersPerSecond())/(1.0*Math.PI))*25);
		}
	}
}
