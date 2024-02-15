package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IDConstants extends BBConstants {
public IDConstants() {
	super("/home/lvuser/IDConstants.ini", true);
	save();
}

public static int topMotor = 14;
public static int bottomMotor = 15;
public static int feedMotor = 16;
public static int groundMotorID = 8;
public static int kickMotorID = 9;
public static int grabMotorID = 17;
public static int FLDriveID = 0;
public static int FLCanCoderID = 30;
public static int FLSteeringID = 1;
public static int FRDriveID = 4;
public static int FRCanCoderID = 33;
public static int FRSteeringID = 5;
public static int BLDriveID = 2;
public static int BLCanCoderID = 31;
public static int BLSteeringID = 3;
public static int BRDriveID = 6;
public static int BRCanCoderID = 32;
public static int BRSteeringID = 7;
public static int leftClimbMotorID = 10;
public static int rightClimbMotorID = 11;
public static int PigeonID = 41;
public static int CANdleID = 40;
public static String CandleCanName = "";
public static String swerveCanbusName = "canivore";
public static int buddyClimbMotorID = 18;
public static int angleCanCoderID = 0;
public static int angleMotorID = 13;
public static int elevatorMotorID = 12;
public static int shooterSensorChannel = 1;
public static int frontSensorChannel = 2;
public static int backSensorChannel = 3;
public static int chamberSensorChannel = 4;
}
