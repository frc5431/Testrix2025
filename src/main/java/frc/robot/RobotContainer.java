// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto.AutoIntakeCoralCommand;
import frc.robot.Commands.Chained.EjectCoralCommand;
import frc.robot.Commands.Chained.ElevatorFeedCommand;
import frc.robot.Commands.Chained.ElevatorPresetCommand;
import frc.robot.Commands.Chained.ElevatorStowCommand;
import frc.robot.Commands.Chained.IntakeCoralCommand;
import frc.robot.Commands.Chained.ScoreCoralCommand;
import frc.robot.Commands.Chained.SmartStowCommand;
import frc.robot.Subsytems.CANdle.TitanCANdle;
import frc.robot.Subsytems.Drivebase.Drivebase;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Feeder;
import frc.robot.Util.Field;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Intake.IntakePivot;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.RobotMechanism;
import frc.robot.Util.TitanBitDoController;
import frc.robot.Util.SwerveConstants;
import frc.robot.Util.Constants.*;
import frc.robot.Util.Constants.CANdleConstants.AnimationTypes;
import frc.robot.Util.Constants.FeederConstants.FeederModes;
import frc.robot.Util.Constants.GameConstants.GamePieceStates;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorStates;
import frc.team5431.titan.core.joysticks.TitanController;

import lombok.Getter;

public class RobotContainer {

	private final @Getter Systems systems = new Systems();
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

	private TitanController driver = Systems.getDriver();
	private TitanController operator = Systems.getOperator();

	private GamePieceStates gamePieceStatus = GamePieceStates.NONE;

	// desired top speed

	private final SwerveRequest.FieldCentric driverControl = new SwerveRequest.FieldCentric()
			.withDeadband(SwerveConstants.kSpeedAt12Volts.times(0.1))
			.withRotationalDeadband(DrivebaseConstants.MaxAngularRate.times(0.1).in(RadiansPerSecond)) // Add a 10%
			.withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
			.withSteerRequestType(SteerRequestType.Position)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	// Triggers

	// Automated Triggers

	// Game Status
	private @Getter Trigger hasAlgae = new Trigger(() -> gamePieceStatus == GamePieceStates.ALGAE);
	private @Getter Trigger hasCoral = new Trigger(() -> gamePieceStatus == GamePieceStates.CORAL);
	private @Getter Trigger isEndgame = new Trigger(
			() -> DriverStation.getMatchTime() <= 20 && DriverStation.isTeleop());
	private @Getter Trigger isAutonEnabled = new Trigger(() -> DriverStation.isAutonomousEnabled());

	// Subsystem Triggers
	private @Getter Trigger isIntaking = new Trigger(
			() -> intake.getMode() == IntakeModes.INTAKE || intake.getMode() == IntakeModes.FEED);

	// LED Triggers
/*// Driver Controls
	// private Trigger alignRightReef = driver.rightBumper();
	// private Trigger alignLeftReef = driver.leftBumper();
	// private Trigger alignCenterReef = driver.a();

	// more Game Status
	// private @Getter Trigger reefAlignment = new Trigger(
	// () -> alignRightReef.getAsBoolean() || alignLeftReef.getAsBoolean());  */
	
	private Trigger zeroDrivebase = driver.y();
	private Trigger driverStow = driver.x();

	

	// Operator Controls

	// Preset Controls
	private Trigger intakePreset = operator.rightBumper();
	private Trigger stowIntake = operator.leftBumper();
	private Trigger smartStow = operator.downDpad();
	private Trigger scoreL2Preset = operator.rightDpad();
	private Trigger scoreL3Preset = operator.leftDpad();
	private Trigger scoreL4Preset = operator.upDpad();

	// Intake Controls
	private Trigger intakeCoral = operator.a();
	private Trigger reverseFeed = operator.b();
	private Trigger smartIntakeCoral = operator.leftTrigger(.5);
	private Trigger scoreCoral = operator.rightTrigger(.5);
	private Trigger manipFeed = operator.back();
	private Trigger killIntake = operator.start();

	public RobotContainer() {
		// Path Planner reccomends that construction of their namedcommands happens
		// before anything else in robot container
		setCommandMappings();
		configureOperatorControls();
		configureDriverControls();

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

	}

	public void subsystemPeriodic() {
		drivebase.periodic();
		intake.periodic();
		feeder.periodic();
		elevator.periodic();
		manipJoint.periodic();
		manipulator.periodic();
		intakePivot.periodic();
	}

	public void periodic() {
		subsystemPeriodic();
		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
		SmartDashboard.putData("mechanism", robotMechanism.elevator);
		gamePieceStatus = (manipulator.getBeambreakStatus()) ? GamePieceStates.CORAL : GamePieceStates.NONE;
		manipulator.setState((manipulator.getBeambreakStatus()) ? ManipulatorStates.LOCKED : ManipulatorStates.EMPTY);

	}

	public double deadzone(double num) {
		if (Math.abs(num) > ControllerConstants.deadzone) {
			return num;
		} else {
			return 0;
		}
	}

	private void configureDriverControls() {

		drivebase.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivebase.applyRequest(
						() -> driverControl
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

		zeroDrivebase.onTrue(new InstantCommand(() -> drivebase.resetGyro())
				.withName("Zero Drivebase"));

	}

	private void configureOperatorControls() {

		// Intake Controls

		stowIntake.onTrue(intakePivot.runIntakePivotCommand(IntakePivotModes.STOW)
				.withName("Stow Intake"));

		intakeCoral.onTrue(
				manipulator.runManipulatorCommand(ManipulatorModes.FEED).until(() -> manipulator.getBeambreakStatus()));

		smartIntakeCoral.whileTrue(
				new ParallelCommandGroup(
						intake.runIntakeCommand(IntakeModes.INTAKE),
						feeder.runFeederCommand(FeederModes.FEED))
								.withName("Smart Intake System"));

		scoreCoral.whileTrue(manipulator.runManipulatorCommand(ManipulatorModes.SCORE)
				.withName("Score Coral"));

		manipFeed.whileTrue(
				manipulator.runManipulatorCommand(ManipulatorModes.SCORE)
						.alongWith(feeder.runFeederCommand(FeederModes.SLOW))
						.withName("Manipulator Feed Command"));

		killIntake.onTrue(intakePivot.killIntakeCommand());

		reverseFeed.whileTrue(new EjectCoralCommand(intake, feeder, manipulator)
				.withName("Coral Outake"));

		// Elevator Controls
		intakePreset.onTrue(
				new ElevatorFeedCommand(elevator, manipJoint)
						.alongWith(intakePivot.runIntakePivotCommand(IntakePivotModes.STOW))
						.withName("Pickup Preset"));

		smartStow.onTrue(
				new SmartStowCommand(elevator, manipJoint, manipulator)
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
		elevator.runOnce(() -> elevator.riseAboveFriction());
		intake.setDefaultCommand(intake.runIntakeCommand(IntakeModes.IDLE).withName("Intake Default Command"));
		feeder.setDefaultCommand(feeder.runFeederCommand(FeederModes.IDLE).withName("Feeder Default Command"));
		manipulator.setDefaultCommand(
				manipulator.runManipulatorCommand(ManipulatorModes.IDLE).withName("Manipulator Default Command"));

		candle.setDefaultCommand(candle.titanCommand().withName("LED Default Command"));

		// Subsystem Status
		// isIntaking.whileTrue(feeder.runFeederCommand(FeederModes.FEED).withName("Feeder
		// Auto Control"));

		// LED Status
		isEndgame.whileTrue(candle.changeAnimationCommand(AnimationTypes.STRESS_TIME).withName("LED Endgame"));
		isAutonEnabled.whileTrue(
				candle.changeAnimationCommand((Field.isRed() ? AnimationTypes.BLINK_RED : AnimationTypes.BLINK_BLUE))
						.withName("LED Auton Alliance"));
		hasCoral.whileTrue(candle.changeAnimationCommand(AnimationTypes.CORAL).withTimeout(1).withName("LEDCoral"));
		hasAlgae.whileTrue(candle.changeAnimationCommand(AnimationTypes.ALGAE));
		hasAlgae.and(hasCoral).onTrue(candle.changeAnimationCommand(AnimationTypes.BOTH));

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
