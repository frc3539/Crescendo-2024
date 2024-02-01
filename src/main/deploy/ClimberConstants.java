package frc.robot.constants;

import org.bytingbulldogs.bulldoglibrary.INIConfiguration.BBConstants;

public class ClimberConstants extends BBConstants {
public ClimberConstants() {
	super("/home/lvuser/ClimberConstants.ini", true);
	save();
}

public static double maxClimbRps = 0.0;
public static double buddyClimbRps = 0.0;
}
