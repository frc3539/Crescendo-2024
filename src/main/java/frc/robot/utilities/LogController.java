package frc.robot.utilities;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class LogController {
//  +-----------------------------------+
//  |  SET ROBOT LOGGING SETTINGS HERE  |
//  +-----------------------------------+

public static boolean USE_LOGGING = true;

static boolean LOG_DRIVE_SUBSYSTEM = true;
static boolean LOG_INTAKE_SUBSYSTEM = true;
static boolean LOG_LEDS = true;

static boolean SAVE_TO_FILE = false;
static String LOG_FILE_PATH = "/media/sda1/";
static boolean USE_NETWORK_TABLES = true;

//  +---------------------------------+
//  |  END OF ROBOT LOGGING SETTINGS  |
//  +---------------------------------+

DrivetrainSubsystem driveSubsystem;
IntakeSubsystem intakeSubsystem;
LedSubsystem leds;

public LogController(DrivetrainSubsystem drive, IntakeSubsystem intake, LedSubsystem leds) {
	if (!USE_LOGGING) return;
	this.driveSubsystem = drive;
	this.intakeSubsystem = intake;
	this.leds = leds;

	Logger.recordMetadata("TeamYear", "FRC3539-2024"); // Set a metadata value

	if (Robot.isReal()) {
	if (SAVE_TO_FILE) Logger.addDataReceiver(new WPILOGWriter(LOG_FILE_PATH));
	if (USE_NETWORK_TABLES) Logger.addDataReceiver(new NT4Publisher());
	} else {
	String logPath =
		LogFileUtil
			.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
	Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
	Logger.addDataReceiver(
		new WPILOGWriter(
			LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
	}

	// logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the
	// "Understanding Data Flow" page
	Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
	// be added.

	DriverStation.startDataLog(DataLogManager.getLog());
}

public void logPeriodic() {
	if (!USE_LOGGING) return;
	logDriveSubsystem();
	logIntakeSubsystem();
}

public void logDriveSubsystem() {
	if (!LOG_DRIVE_SUBSYSTEM) return;
	driveSubsystem.log();
}

public void logIntakeSubsystem() {
	if (!LOG_INTAKE_SUBSYSTEM) return;
	intakeSubsystem.log();
}
//Cameron is a goon
}
