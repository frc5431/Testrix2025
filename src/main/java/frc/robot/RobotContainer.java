// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Util.Field;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto.AutoIntakeCoralCommand;
import frc.robot.Commands.Chained.EjectCoralCommand;
import frc.robot.Commands.Chained.ElevatorFeedCommand;
import frc.robot.Commands.Chained.ElevatorPresetCommand;
import frc.robot.Commands.Chained.ElevatorStowCommand;
import frc.robot.Commands.Chained.IntakeCoralCommand;
import frc.robot.Commands.Chained.ScoreCoralCommand;
import frc.robot.Commands.Chained.SmartStowCommand;
import frc.robot.Commands.Chained.ZeroCommand;
import frc.robot.Subsytems.CANdle.TitanCANdle;
import frc.robot.Subsytems.Drivebase.Drivebase;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Feeder;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Intake.IntakePivot;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.RobotMechanism;
import frc.robot.Util.SwerveConstants;
import frc.robot.Util.Constants.*;
import frc.robot.Util.Constants.CANdleConstants.AnimationTypes;
import frc.robot.Util.Constants.FeederConstants.FeederModes;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotModes;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;
import frc.team5431.titan.core.joysticks.TitanController;

import lombok.Getter;

public class RobotContainer {

	private final Systems systems = new Systems();
	private final RobotMechanism robotMechanism = new RobotMechanism();
	private final Intake intake = systems.getIntake();
	private final IntakePivot intakePivot = systems.getIntakePivot();
	private final Feeder feeder = systems.getFeeder();
	private final Elevator elevator = systems.getElevator();
	private final ManipJoint manipJoint = systems.getManipJoint();
	private final Manipulator manipulator = systems.getManipulator();
	private final TitanCANdle candle = Systems.getTitanCANdle();
	private final Drivebase drivebase = Systems.getDrivebase();
	private final SendableChooser<Command> autoChooser;
	private final SendableChooser<Boolean> enable = new SendableChooser<>();

	private TitanController driver = Systems.getDriver();
	private TitanController operator = Systems.getOperator();

	// Triggers

	// Automated Triggers

	// Game Status
	private @Getter Trigger isEndgame = new Trigger(
			() -> DriverStation.getMatchTime() <= 5 && DriverStation.isTeleop());
	private @Getter Trigger isAutonEnabled = new Trigger(() -> DriverStation.isAutonomousEnabled());

	// Subsystem Triggers
	private @Getter Trigger isIntaking = new Trigger(
			() -> intake.getMode() == IntakeModes.INTAKE || intake.getMode() == IntakeModes.FEED);

	private @Getter Trigger isManipIntaking = new Trigger(
			() -> manipulator.getMode() == ManipulatorModes.MANUAL);

	// LED Triggers
	/*
	 * // Driver Controls
	 * // private Trigger alignRightReef = driver.rightBumper();
	 * // private Trigger alignLeftReef = driver.leftBumper();
	 * // private Trigger alignCenterReef = driver.a();
	 * 
	 * // more Game Status
	 * // private @Getter Trigger reefAlignment = new Trigger(
	 * // () -> alignRightReef.getAsBoolean() || alignLeftReef.getAsBoolean());
	 */
	private Trigger enabledGUI = new Trigger(() -> enable.getSelected());

	private Trigger zeroDrivebase = driver.y();
	private Trigger driverStow = driver.x();
	private Trigger killElevator = driver.b();
	private Trigger driverIntake = driver.leftTrigger(0.5);
	private Trigger di = driver.rightTrigger(0.5);
	// private Trigger povUp
	// private Trigger povUpRight = driver.
	// private Trigger

	// Operator Controls

	// Preset Controls
	private Trigger feedPreset = operator.downDpad();
	private Trigger scoreL2Preset = operator.rightDpad();
	private Trigger scoreL3Preset = operator.leftDpad();
	private Trigger scoreL4Preset = operator.upDpad();
	private Trigger ejectL4Preset = operator.y();
	private Trigger stowPreset = operator.start();
	private Trigger intakePreset = operator.rightBumper();

	// Intake Controls
	private Trigger intakeCoral = operator.a();
	private Trigger reverseFeed = operator.b();
	private Trigger zeroElevator = operator.back();
	private Trigger stowIntake = operator.leftBumper();
	private Trigger manipFeed = operator.x();

	private Trigger smartIntakeCoral = operator.leftTrigger(.5);
	private Trigger scoreCoral = operator.rightTrigger(.5);

	public RobotContainer() {
		// Path Planner reccomends that construction of their namedcommands happens
		// before anything else in robot container
		setCommandMappings();
		configureOperatorControls();
		configureDriverControls();
		new SmartStowCommand(elevator, manipJoint, manipulator).schedule();

		System.out.println(AutoBuilder.getAllAutoNames());
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		enable.addOption("true", true);
		enable.addOption("false", false);
		SmartDashboard.putData("Enable GUI", enable);
		candle.testCommand().runsWhenDisabled();
	}

	public void subsystemPeriodic() {
		drivebase.periodic();
		intake.periodic();
		feeder.periodic();
		elevator.periodic();
		manipJoint.periodic();
		manipulator.periodic();
		intakePivot.periodic();
		candle.periodic();
	}

	public void periodic() {
		subsystemPeriodic();
		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
		SmartDashboard.putData("mechanism", robotMechanism.elevator);
	}

	/**
	 * Sets a Deazone
	 * Make a linear function with deadson at 0 and 1 at 1.
	 * Then need to have this work on both positive and negative.
	 * 
	 * @param num
	 * @return
	 */
	public double deadzone(double num) {
		if (Math.abs(num) > ControllerConstants.deadzone) {

			double w = 1.0 / (1.0 - ControllerConstants.deadzone);
			double b = w * ControllerConstants.deadzone;
			return (w * Math.abs(num) - b) * (num / Math.abs(num));
		} else {
			return 0;
		}
	}

	private void configureDriverControls() {
		drivebase.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivebase.applyRequest(
						() -> drivebase.getDriverControl()
								.withVelocityX(deadzone(-driver.getLeftY())
										* SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond))
								.withVelocityY(deadzone(-driver.getLeftX())
										* SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond))
								.withRotationalRate(
										deadzone(-driver.getRightX()
												* DrivebaseConstants.MaxAngularRate.in(RadiansPerSecond))))
						.withName("Swerve Default Command"));

		// Align Reef Commands
		// alignLeftReef.onTrue(
		// new AlignReefCommand(false).withName("Align Left Reef"));
		// alignRightReef.onTrue(
		// new AlignReefCommand(true).withName("Align Right Reef"));
		// alignCenterReef.onTrue(
		// new AlignReefCommand().withName("Align Center Reef"));

		driverStow.onTrue(
				new SmartStowCommand(elevator, manipJoint, manipulator)
						.alongWith(intakePivot.runIntakePivotCommand(IntakePivotModes.STOW))
						.withName("Driver Smart Stow"));

		killElevator.onTrue(manipJoint.killManipJoingCommand().alongWith(elevator.killElevatorCommand()));

		zeroDrivebase.onTrue(new InstantCommand(() -> drivebase.resetGyro())
				.withName("Zero Drivebase"));

		driverIntake.whileTrue(intake.runIntakeCommand(IntakeModes.INTAKE));
		di.whileTrue(intake.runIntakeCommand(IntakeModes.INTAKE));

	}

	private void configureOperatorControls() {

		// Intake Controls
		ejectL4Preset.onTrue(new ElevatorPresetCommand(ControllerConstants.ejectL4, elevator, manipJoint));
		zeroElevator.onTrue(new ZeroCommand(elevator).withName("ZERO ELEVATOR MAY WANT KILL"));

		stowIntake.onTrue(intakePivot.runIntakePivotCommand(IntakePivotModes.STOW)
				.withName("Stow Intake"));

		intakeCoral.onTrue(
				manipulator.runManipulatorCommand(ManipulatorModes.FEED).until(() -> manipulator.hasCoral()));

		smartIntakeCoral.whileTrue(
				intake.runIntakeCommand(IntakeModes.INTAKE).withName("Smart Intake System"));

		scoreCoral.whileTrue(manipulator.runManipulatorCommand(ManipulatorModes.SCORE)
				.withName("Score Coral"));

		manipFeed.whileTrue(
				manipulator.runManipulatorCommand(ManipulatorModes.MANUAL)
						.alongWith(feeder.runFeederCommand(FeederModes.SLOW))
						.withName("Manipulator Feed Command"));

		stowPreset.onTrue(new ElevatorStowCommand(elevator, manipJoint)
				.withName("Smart Stow"));

		reverseFeed.whileTrue(new EjectCoralCommand(intake, feeder, manipulator)
				.withName("Coral Outake"));

		// Elevator Controls
		intakePreset.onTrue(
				intakePivot.runIntakePivotCommand(IntakePivotModes.DEPLOY)
						.withName("Pickup Preset"));

		feedPreset.onTrue(
				new ElevatorFeedCommand(elevator, manipJoint)
						.withName("Smart Stow"));

		scoreL2Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL2Position, elevator, manipJoint)
						.withName("Elevator L2 Preset"));

		scoreL3Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL3Position, elevator, manipJoint)
						.withName("Elevator L3 Preset"));

		scoreL4Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL4Position, elevator, manipJoint)
						.withName("Elevator L4 Preset"));

	}

	public void onInitialize() {
		// Default Commands
		manipJoint.runManipJointCommand(ManipJointPositions.STOW);
		elevator.runOnce(() -> elevator.riseAboveFriction());
		intake.setDefaultCommand(intake.runIntakeCommand(IntakeModes.IDLE).withName("Intake Default Command"));
		feeder.setDefaultCommand(feeder.runFeederCommand(FeederModes.IDLE).withName("Feeder Default Command"));
		manipulator.setDefaultCommand(
				manipulator.runManipulatorCommand(ManipulatorModes.IDLE).withName("Manipulator Default Command"));

		candle.setDefaultCommand(candle.testCommand().withName("LED Default Command"));

		// // Subsystem Status
		isIntaking.whileTrue(feeder.runFeederCommand(FeederModes.FEED).withName("Feeder Auto Control"));
		isManipIntaking.whileTrue(feeder.runFeederCommand(FeederModes.SLOW));

		// // LED Status
		// isEndgame.whileTrue(candle.changeAnimationCommand(AnimationTypes.STRESS_TIME).withName("LED
		// Endgame"));
		// isAutonEnabled.whileTrue(
		// candle.changeAnimationCommand((Field.isRed() ? AnimationTypes.BLINK_RED :
		// AnimationTypes.BLINK_BLUE))
		// .withName("LED Auton Alliance"));

	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void setCommandMappings() {
		NamedCommands.registerCommand("EjectCoral",
				new EjectCoralCommand(intake, feeder, manipulator));
		NamedCommands.registerCommand("ElevatorFeed",
				new ElevatorFeedCommand(elevator, manipJoint));
		NamedCommands.registerCommand("L1Preset",
				new ElevatorPresetCommand(ControllerConstants.ScoreL1Position, elevator, manipJoint));
		NamedCommands.registerCommand("L2Preset",
				new ElevatorPresetCommand(ControllerConstants.ScoreL2Position, elevator, manipJoint));
		NamedCommands.registerCommand("L3Preset",
				new ElevatorPresetCommand(ControllerConstants.ScoreL3Position, elevator, manipJoint));
		NamedCommands.registerCommand("L4Preset",
				new ElevatorPresetCommand(ControllerConstants.ScoreL4Position, elevator, manipJoint));
		NamedCommands.registerCommand("StowPreset",
				new ElevatorStowCommand(elevator, manipJoint));
		NamedCommands.registerCommand("SimpleScore",
				manipulator.runManipulatorCommand(ManipulatorModes.SCORE));
		NamedCommands.registerCommand("ScoreL4",
				new ParallelCommandGroup(manipulator.runManipulatorCommand(ManipulatorModes.SCORE),
						new WaitCommand(2).andThen(manipJoint.runManipJointCommand(ManipJointPositions.STOW))));
		NamedCommands.registerCommand("IntakeCoral",
				new IntakeCoralCommand(intake, intakePivot, manipulator, elevator, manipJoint));
		NamedCommands.registerCommand("AutoIntakeCoral",
				new AutoIntakeCoralCommand(intake, intakePivot, manipulator, elevator, manipJoint));
		NamedCommands.registerCommand("ScoreCoral",
				new ScoreCoralCommand(elevator, manipJoint, manipulator));
		// NamedCommands.registerCommand("AlignLeftReef", new AlignReefCommand(false));
		// NamedCommands.registerCommand("AlignRightReef", new AlignReefCommand(true));

	}
}
