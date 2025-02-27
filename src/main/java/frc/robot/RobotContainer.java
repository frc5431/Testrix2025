// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto.AutoIntakeCoralCommand;
import frc.robot.Commands.Chained.EjectCoralCommand;
import frc.robot.Commands.Chained.ElevatorFeedCommand;
import frc.robot.Commands.Chained.ElevatorPresetCommand;
import frc.robot.Commands.Chained.ElevatorStowCommand;
import frc.robot.Commands.Chained.IntakeCoralCommand;
import frc.robot.Commands.Chained.ScoreCoralCommand;
import frc.robot.Subsytems.CANdle.TitanCANdle;
import frc.robot.Subsytems.Climber.Climber;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Feeder;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Intake.IntakePivot;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Field;
import frc.robot.Util.RobotMechanism;
import frc.robot.Util.TitanBitDoController;
import frc.robot.Util.Constants.*;
import frc.robot.Util.Constants.CANdleConstants.AnimationTypes;
import frc.robot.Util.Constants.ClimberConstants.ClimberModes;
import frc.robot.Util.Constants.FeederConstants.FeederModes;
import frc.robot.Util.Constants.GameConstants.GamePieceStates;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
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
	private final Climber climber = systems.getClimber();
	private @Getter final TitanCANdle candle = Systems.getTitanCANdle();

	private TitanController driver = new TitanController(ControllerConstants.driverPort, ControllerConstants.deadzone);
	private TitanController operator = new TitanController(ControllerConstants.operatorPort,
			ControllerConstants.deadzone);
	private TitanBitDoController operator8BitDo = new TitanBitDoController(ControllerConstants.operatorPort);

	private GamePieceStates gamePieceStatus = GamePieceStates.NONE;

	private final SendableChooser<Command> autoChooser;

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

	// Driver Controls
	private Trigger alignRightReef = driver.rightBumper();
	private Trigger alignLeftReef = driver.leftBumper();

	// Climber Controls
	private Trigger climberOut = driver.leftTrigger(ControllerConstants.triggerThreshold);
	private Trigger climberClimb = driver.rightTrigger(ControllerConstants.triggerThreshold);

	// more Game Status
	private @Getter Trigger reefAlignment = new Trigger(() -> alignRightReef.getAsBoolean() || alignLeftReef.getAsBoolean());

	// Operator Controls

	// Preset Controls
	private Trigger intakePreset = ControllerConstants.using8BitDo ? operator8BitDo.getLBumper() : operator.leftBumper();
	private Trigger processorPreset = ControllerConstants.using8BitDo ? operator8BitDo.getMinus() : operator.back();
	private Trigger feedPreset = ControllerConstants.using8BitDo ? operator8BitDo.getPovRight() : operator.rightDpad();
	private Trigger scoreL2Preset = ControllerConstants.using8BitDo ? operator8BitDo.getPovDown() : operator.downDpad();
	private Trigger scoreL3Preset = ControllerConstants.using8BitDo ? operator8BitDo.getPovLeft() : operator.leftDpad();
	private Trigger scoreL4Preset = ControllerConstants.using8BitDo ? operator8BitDo.getPovUp() : operator.upDpad();

	// Cleaner Controls
	private Trigger intakeAlgae = ControllerConstants.using8BitDo ? operator8BitDo.getB() : operator.b();
	private Trigger outtakeAlgae = ControllerConstants.using8BitDo ? operator8BitDo.getX() : operator.x();

	// Intake Controls
	private Trigger intakeCoral = ControllerConstants.using8BitDo ? operator8BitDo.getA() : operator.a();
	private Trigger scoreCoral = ControllerConstants.using8BitDo ? operator8BitDo.getY() : operator.y();
	private Trigger reverseFeed = ControllerConstants.using8BitDo ? operator8BitDo.getRightDPadDown() : operator.rightStick();

	public RobotContainer() {
		configureBindings();

		setCommandMappings();

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public void subsystemPeriodic() {
		intake.periodic();
		feeder.periodic();
		climber.periodic();
		elevator.periodic();
		manipJoint.periodic();
		manipulator.periodic();
		intakePivot.periodic();
		
	}

	public void periodic() {
		subsystemPeriodic();
		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
		SmartDashboard.putData("mechanism", robotMechanism.elevator);
		SmartDashboard.putBoolean("Operator Controller", ControllerConstants.using8BitDo);
		SmartDashboard.putBoolean("Intake Algae (Test)", intakeAlgae.getAsBoolean());
		gamePieceStatus = (manipulator.getBeambreakStatus()) ? GamePieceStates.CORAL : GamePieceStates.NONE;
		manipulator.setState((manipulator.getBeambreakStatus()) ? ManipulatorStates.LOCKED : ManipulatorStates.EMPTY);

	}

	private void configureDriverControls() {

		// Climber Controls
		climberOut.onTrue(climber.runClimberCommand(ClimberModes.ALIGN));
		climberClimb.whileTrue(climber.runClimberCommand(ClimberConstants.climbVelocity));

	}

	private void configureOperatorControls() {

		// Elevator Controls
		intakePreset.onTrue(
				new IntakeCoralCommand(intake, intakePivot, manipulator, elevator, manipJoint)
						.withName("Intake Coral Preset"));

		processorPreset.onTrue(
				new ElevatorStowCommand(elevator, manipJoint)
						.withName("Elevator Algae Intake"));

		feedPreset.onTrue(
				new ElevatorFeedCommand(elevator, manipJoint)
						.withName("Elevator Feed Positon"));

		scoreL2Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL2Position, elevator, manipJoint)
						.withName("Elevator L2 Preset"));

		scoreL3Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL3Position, elevator, manipJoint)
						.withName("Elevator L3 Preset"));

		scoreL4Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL4Position, elevator, manipJoint)
						.withName("Elevator L4 Preset"));

		// Intake Controls
		intakeCoral.whileTrue(new ParallelCommandGroup(intake.runIntakeCommand(IntakeModes.INTAKE),
				(manipulator.runManipulatorCommand(ManipulatorModes.FEED))).withName("Run Intake System"));
		scoreCoral.whileTrue(manipulator.runManipulatorCommand(ManipulatorModes.REVERSE).withName("Score Coral"));
		reverseFeed.whileTrue(new EjectCoralCommand(intake, feeder, manipulator).withName("Coral Outake"));

	}

	public void configureBindings() {

		configureOperatorControls();
		configureDriverControls();
	}

	public void onInitialize() {
		// Default Commands
		candle.setDefaultCommand(candle.titanCommand().withName("LED Default Command"));
		climber.runClimberCommand(ClimberModes.STOW);

		// Subsystem Status
		isIntaking.whileTrue(feeder.runFeederCommand(FeederModes.FEED).withName("Feeder Auto Control"));

		// LED Status
		isEndgame.whileTrue(candle.changeAnimationCommand(AnimationTypes.STRESS_TIME).withName("LED Endgame"));
		isAutonEnabled.whileTrue(
				candle.changeAnimationCommand((Field.isRed() ? AnimationTypes.BLINK_RED : AnimationTypes.BLINK_BLUE))
						.withName("LED Auton Alliance"));
		hasCoral.whileTrue(candle.changeAnimationCommand(AnimationTypes.CORAL).withTimeout(1).withName("LED Coral"));
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

	}
}
