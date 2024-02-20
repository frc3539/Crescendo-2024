package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class ShooterConstants extends BBConstants {
public ShooterConstants() {
	super("/home/lvuser/ShooterConstants.ini", true);
	save();
}

public static double shootDps = 1000;
public static double feedDps = 1000;
public static double elevatorDps = 0.0;
public static double angleShooterP = 420;
public static double angleShooterI = 0.0;
public static double angleShooterD = 0.0;
public static double angleShooterV = 12;
public static double angleShooterG = 0.24;
public static double angleShooterSoftMin = -0.118;
public static double angleShooterSoftMax = 0.25;
public static double elevatorMotorP = 12;
public static double elevatorMotorI = 0.0;
public static double elevatorMotorD = 0.0;
public static double elevatorMotorV = 0.125;
public static double elevatorMotorG = 0.348;
public static double elevatorSoftMin = 0;
public static double elevatorSoftMax = 18;
public static double angleMotorToEncoder = 94.5; // Gear ratio between motor and piviot shaft
public static double elevatorMotorToInches = 0.1598888; // ins. per revolution
public static double shooterAngleOffset = -0.118652;
public static double minElevatorMoveAngle = 0.0;
public static double shootP = 0.25;
public static double shootV = 0.125;
public static double shootWheelDiameter = 4;
public static double feedWheelDiameter = 4;
public static double feedP = 0.25;
public static double feedV = 0.125;
public static double restShooterAngle = -30; // degrees
public static double elevatorCollisionHeight = 5; // in inches
public static double ampDps = 250;
}
