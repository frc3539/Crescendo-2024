package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IDConstants extends BBConstants {
public IDConstants() {
	super("/home/lvuser/IDConstants.ini", true);
	save();
}

public static int topMotor = 3;
public static int bottomMotor = 4;
public static int feedMotor = 5;
public static int groundMotorID = 0;
public static int kickMotorID = 0;
public static int grabMotorID = 0;
public static int FLDriveID = 0;
public static int FLCanCoderID = 0;
public static int FLSteeringID = 0;
public static int FRDriveID = 0;
public static int FRCanCoderID = 0;
public static int FRSteeringID = 0;
public static int BLDriveID = 0;
public static int BLCanCoderID = 0;
public static int BLSteeringID = 0;
public static int BRDriveID = 0;
public static int BRCanCoderID = 0;
public static int BRSteeringID = 0;
public static int leftClimbMotorID = 0;
public static int rightClimbMotorID = 0;
public static int PigeonID = 0;
public static int CANdleID = 0;
public static String CandleCanName = "";
public static String swerveCanbusName = "canivore";
public static int buddyClimbMotorID = 0;
public static int angleCanCoderID = 0;
public static int angleMotorID = 0;
public static int elevatorMotorID = 0;
public static int shooterSensorChannel = 1;
public static int frontSensorChannel = 2;
public static int backSensorChannel = 3;
public static int chamberSensorChannel = 4;
}