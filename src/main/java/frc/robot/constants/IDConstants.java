package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IDConstants extends BBConstants{
	public IDConstants() {
		super("/home/lvuser/IDConstants.ini", true);
		save();
	}
	public static int topMotor = 0;
	public static int bottomMotor = 0;
	public static int feedMotor = 0;
	public static int intakeMotorOne = 0;
	public static int intakeMotorTwo = 0;
	public static int intakeMotorThree = 0;
}
