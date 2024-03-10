// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.IDConstants;
import frc.robot.constants.LedConstants;

public class LedSubsystem extends SubsystemBase {

	boolean intaking;
	boolean enabled;
	CANdle candle;

	public LedSubsystem(boolean enabled) {
		this.enabled = enabled;

		this.candle = new CANdle(IDConstants.CANdleID, IDConstants.CandleCanName);
		candle.configLEDType(LEDStripType.GRB);
		candle.configBrightnessScalar(LedConstants.maxBrightness);
		candle.animate(null);
		candle.setLEDs(0, 255, 0, 0, 0, LedConstants.numLights);
	}

	public enum LEDState {
		ON, OFF, READY, INTAKING, INTAKING_EMPTY, SHOOTING, PREPARED, CLIMBING, AUTO
	}

	public LEDState state;

	public void setLEDs(LEDState state) {
		if (!enabled)
			return;
		switch (state) {
			case OFF :
				candle.animate(null);
				candle.setLEDs(0, 0, 0);
				break;
			case ON :
				candle.animate(null);
				candle.setLEDs(LedConstants.Green.getRed(), LedConstants.Green.getGreen(),
						LedConstants.Green.getBlue());

				break;

			case READY :
				candle.animate(new LarsonAnimation(LedConstants.Green.getRed(), LedConstants.Green.getGreen(),
						LedConstants.Green.getBlue(), 0, 0.5, LedConstants.numLights, BounceMode.Back, 6));
				break;
			case INTAKING :
				candle.animate(new StrobeAnimation(LedConstants.Orange.getRed(), LedConstants.Orange.getGreen(),
						LedConstants.Orange.getBlue(), 0, LedConstants.flashSpeed, LedConstants.numLights));
				break;
			case INTAKING_EMPTY :
				candle.animate(new StrobeAnimation(0, 255, 0, 0, LedConstants.flashSpeed, LedConstants.numLights));
				break;

			case SHOOTING :
				candle.animate(new ColorFlowAnimation(LedConstants.Orange.getRed(), LedConstants.Orange.getGreen(),
						LedConstants.Orange.getBlue(), 0, LedConstants.flashSpeed, LedConstants.numLights, null));
				break;

			case PREPARED :
				candle.animate(null);
				candle.setLEDs(LedConstants.Orange.getRed(), LedConstants.Orange.getGreen(),
						LedConstants.Orange.getBlue(), 0, 0, LedConstants.numLights);

				break;

			case CLIMBING :
				break;

			case AUTO :
				candle.animate(new StrobeAnimation(LedConstants.Blue.getRed(), LedConstants.Blue.getGreen(),
						LedConstants.Blue.getBlue(), 0, LedConstants.flashSpeed, LedConstants.numLights));
				break;

			default :
				break;
		}
	}
	public void setIntaking(boolean intaking) {
		this.intaking = intaking;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// setLEDs(LEDState.READY);
		if (RobotContainer.shooterSubsystem.getShooterSensor()) {
			setLEDs(LEDState.PREPARED);
			return;
		} else {
			if (RobotContainer.intakeSubsystem.getChamberSensor()) {
				setLEDs(LEDState.INTAKING);
				return;
			}
		}
		if (this.intaking) {
			if (RobotContainer.intakeSubsystem.getChamberSensor()) {

				setLEDs(LEDState.INTAKING);
				return;
			} else {
				setLEDs(LEDState.INTAKING_EMPTY);
				return;
			}
		}

		setLEDs(LEDState.ON);
	}
}
