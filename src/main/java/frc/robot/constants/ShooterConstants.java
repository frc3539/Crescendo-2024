package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ShooterConstants extends BBConstants {
public ShooterConstants() {
	super("/home/lvuser/ShooterConstants.ini", true);
	save();
}

public static double revRps = 0.0;
public static double shootRps = 0.0;
public static double angleShooterP = 0.0;
public static double angleShooterI = 0.0;
public static double angleShooterD = 0.0;
public static double angleShooterV = 0.0;
public static double angleShooterG = 0.0;
public static int angleShooterSoftMin = 0;
public static int angleShooterSoftMax = 0;
}
