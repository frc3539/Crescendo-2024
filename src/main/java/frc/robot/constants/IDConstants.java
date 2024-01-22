package frc.robot.constants;

import org.frcteam3539.BulldogLibrary.INIConfiguration.BBConstants;

public class IDConstants extends BBConstants {
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
  public static int PigeonID = 0;
  public static String swerveCanbusName = "canivore";
}
