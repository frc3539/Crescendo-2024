// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.constants.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

public static DrivetrainConstant drivetrainConstants = new DrivetrainConstant();
public static ClimberConstants climberConstants = new ClimberConstants();
public static IDConstants idConstants = new IDConstants();
public static IntakeConstants intakeConstants = new IntakeConstants();
public static ShooterConstants shooterConstants = new ShooterConstants();
public static VisionConstants visionConstants = new VisionConstants();

public static DrivetrainSubsystem drivetrainSubsystem = TunerConstants.DriveTrain;
public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
// public static LedSubsystem ledSubsystem = new LedSubsystem(true);
public static VisionSubsystem visionSubsystem = new VisionSubsystem();
public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public static CommandXboxController driverController = new CommandXboxController(1);

public static CommandXboxController operatorController = new CommandXboxController(0);

public RobotContainer() {
	// Configure the trigger bindings
	configureBindings();
}

/**
* Use this method to define your trigger->command mappings. Triggers can be created via the
* {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
* predicate, or via the named factories in {@link
* edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
* CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
* PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
* joysticks}.
*/
private void configureBindings() {
	operatorController.leftBumper().whileTrue(new RevUpCommand());
	operatorController.rightBumper().whileTrue(new ShootCommand());
	operatorController.povUp().whileTrue(new IntakeCommand(true, IntakeMode.FRONT));
	operatorController.povDown().whileTrue(new IntakeCommand(true, IntakeMode.BACK));
	operatorController.y().whileTrue(new IntakeCommand(true, IntakeMode.SENSOR));
	operatorController.leftTrigger().whileTrue(new IndependantClimbLeftCommand());
	operatorController.rightTrigger().whileTrue(new IndependantClimbRightCommand());
}

/**
* Use this to pass the autonomous command to the main {@link Robot} class.
*
* @return the command to run in autonomous
*/
public Command getAutonomousCommand() {
	// An example command will be run in autonomous
	return null;
}
}
