// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autons.Blue4Piece;
import frc.robot.autons.BlueShootDrive;
import frc.robot.commands.*;
import frc.robot.commands.AutoAlignCommand.TagPosition;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.constants.*;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsComp;
import frc.robot.subsystems.*;
import frc.robot.utilities.LogController;
import frc.robot.subsystems.LedSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	public static LogController logController = new LogController();

	public static DrivetrainConstants drivetrainConstants = new DrivetrainConstants();
	public static ClimberConstants climberConstants = new ClimberConstants();
	public static IDConstants idConstants = new IDConstants();
	public static IntakeConstants intakeConstants = new IntakeConstants();
	public static ShooterConstants shooterConstants = new ShooterConstants();
	public static VisionConstants visionConstants = new VisionConstants();

	public static TunerConstants tunerConstants = new TunerConstants();

	public static CommandSwerveDrivetrain drivetrainSubsystem = TunerConstantsComp.DriveTrain; // TunerConstants.DriveTrain
	public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

	public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
	public static LedSubsystem ledSubsystem = new LedSubsystem(true);
	public static VisionSubsystem visionSubsystem = new VisionSubsystem();
	public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public static CommandXboxController driverController = new CommandXboxController(1);

	public static CommandXboxController operatorController = new CommandXboxController(0);

	public static Trigger rightDriverTrigger = driverController.rightTrigger(0.5);
	public static Trigger rightDriverBumper = driverController.rightBumper();
	public static Trigger driverButtonA = driverController.a();

	public static SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		putCommands();
		putAutons();
		visionSubsystem.start();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
	 * for {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
	 * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
	 * Flight joysticks}.
	 *
	 */
	private void putAutons() {
		chooser.setDefaultOption("Blue Shoot and Drive", new BlueShootDrive());
		chooser.addOption("Blue 4 Piece Center", new Blue4Piece());

		SmartDashboard.putData(chooser);
	}
	private void configureBindings() {
		operatorController.leftBumper().whileTrue(new RevUpCommand(false, ShooterConstants.shootDps));
		operatorController.rightBumper().whileTrue(new ShootCommand());
		// operatorController.povUp().whileTrue(new IntakeCommand());
		operatorController.povUp().whileTrue(new IntakeCommand(true, IntakeMode.FRONT));
		operatorController.povDown().whileTrue(new IntakeCommand(true, IntakeMode.BACK));
		operatorController.a().whileTrue(new IntakeCommand(true, IntakeMode.SENSOR));
		operatorController.back().whileTrue(new IntakeCommand(false, IntakeMode.BACK));

		operatorController.leftTrigger(.1).whileTrue(new IndependantClimbLeftCommand());
		operatorController.rightTrigger(.1).whileTrue(new IndependantClimbRightCommand());

		operatorController.b().whileTrue(new AutoShootCommand().finallyDo(() -> {
			CommandScheduler.getInstance().schedule(new HomePositionCommand());
		}));
		// operatorController.a().whileTrue(new AngleShooterCommand(-29.5));
		operatorController.y().whileTrue(new AutoClimbCommand());
		// operatorController.y().onTrue(new SetElevatorCommand(8));
		operatorController.x().whileTrue(new AmpCommand().finallyDo(() -> {
			CommandScheduler.getInstance().schedule(new HomePositionCommand());
		}));
		operatorController.povLeft().onTrue(new AngleShooterCommand(55));
		operatorController.povRight().onTrue(new HomePositionCommand());

		// operatorController.start().whileTrue(new BuddyClimbCommand());
		driverController.start().whileTrue(new ZeroGyroCommand());
		driverController.y().whileTrue(new AutoAlignCommand(TagPosition.TRAP));

		drivetrainSubsystem.setDefaultCommand(new DriveCommand());
	}

	public void putCommands() {
		SmartDashboard.putData(new DisableArmBreakModeCommand().ignoringDisable(true));
		SmartDashboard.putData(new DisableElevatorBreakModeCommand().ignoringDisable(true));
		SmartDashboard.putData(new DisableClimberBreakModeCommand().ignoringDisable(true));

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return chooser.getSelected();
	}
}
