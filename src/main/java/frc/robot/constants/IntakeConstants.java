package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IntakeConstants extends BBConstants{
public IntakeConstants() {
	super("/home/lvuser/IntakeConstants.ini", true);
	save();
}
public static double intakeRps = 0;
public static double kickRps = 0;
}