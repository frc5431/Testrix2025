// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Chained.EjectCoralCommand;
import frc.robot.Commands.Chained.ElevatorPresetCommand;
import frc.robot.Commands.Chained.ElevatorStowCommand;
import frc.robot.Subsytems.CANdle.TitanCANdle;
import frc.robot.Subsytems.Cleaner.CleanPivot;
import frc.robot.Subsytems.Cleaner.Cleaner;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Feeder;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.*;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotModes;
import frc.robot.Util.Constants.CleanerConstants.CleanerModes;
import frc.robot.Util.Constants.GamePieceConstants.GamePieceStates;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;
import frc.team5431.titan.core.joysticks.TitanController;

public class RobotContainer {

	private final Systems systems = new Systems();
	
	private final Intake intake = systems.getIntake();
	private final Feeder feeder = systems.getFeeder();
	private final Cleaner cleaner = systems.getCleaner();
	private final Elevator elevator = systems.getElevator();
	private final CleanPivot cleanPivot = systems.getCleanPivot();
	private final ManipJoint manipJoint = systems.getManipJoint();
	private final Manipulator manipulator = systems.getManipulator();
	private final TitanCANdle candle = systems.getCandle();

	private TitanController driver = new TitanController(ControllerConstants.driverPort, ControllerConstants.deadzone);
	private TitanController operator = new TitanController(ControllerConstants.operatorPort,
			ControllerConstants.deadzone);

	private final GamePieceStates gamePieceStatus = GamePieceStates.NONE;

	// Triggers

	// Gamepiece Status
	private Trigger hasAlgae = new Trigger(() -> gamePieceStatus == GamePieceStates.ALGAE);
	private Trigger hasCoral = new Trigger(() -> gamePieceStatus == GamePieceStates.CORAL);
	
	// LED Triggers

	// Driver Controls
	private Trigger climberOut = driver.rightBumper();

	// Operator Controls

	// Preset Controls
	private Trigger processorPreset = operator.back();
	private Trigger stowPreset = operator.rightDpad();
	private Trigger scoreL2Preset = operator.downDpad();
	private Trigger scoreL3Preset = operator.leftDpad();
	private Trigger scoreL4Preset = operator.upDpad();

	// Cleaner Controls
	private Trigger intakeAlgea = operator.b();
	private Trigger outtakeAlgea = operator.x();

	// Intake Controls
	private Trigger intakeCoral = operator.a();
	private Trigger scoreCoral = operator.y();
	private Trigger reverseFeed = operator.rightStick();
	
	public RobotContainer() {
		configureBindings();
	}

	public void periodic() {
		intake.periodic();
		cleaner.periodic();
		elevator.periodic();
		cleanPivot.periodic();
		manipJoint.periodic();
		manipulator.periodic();
		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

	}

	private void configureDriverControls() {

	}

	private void configureOperatorControls() {

		// Elevator Controls
		processorPreset.onTrue(  
				new ElevatorStowCommand(CleanPivotModes.INTAKE, elevator, manipJoint, cleanPivot)
						.withName("Elevator Algea Intake"));

		stowPreset.onTrue(
				new ElevatorStowCommand(CleanPivotModes.STOW, elevator, manipJoint, cleanPivot)
						.withName("Elevator Stow"));

		scoreL2Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL2Position, elevator, manipJoint, cleanPivot)
						.withName("Elevator L2"));

		scoreL3Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL3Position, elevator, manipJoint, cleanPivot)
						.withName("Elevator L3"));

		scoreL4Preset.onTrue(
				new ElevatorPresetCommand(ControllerConstants.ScoreL4Position, elevator, manipJoint, cleanPivot)
						.withName("Elevator L4"));

		// Intake Controls
		intakeCoral.whileTrue(intake.runIntakeCommand(IntakeModes.INTAKE));
		scoreCoral.whileTrue(manipulator.runManipulatorCommand(ManipulatorModes.REVERSE).withName("Score Coral"));
		reverseFeed.whileTrue(new EjectCoralCommand(intake, feeder, manipulator).withName("Coral Outake"));

		// Cleaner Controls
		intakeAlgea.whileTrue(cleaner.runCleanerCommand(CleanerModes.INTAKE).withName("Intake Algea"));
		outtakeAlgea.whileTrue(cleaner.runCleanerCommand(CleanerModes.OUTTAKE).withName("Outtake Algea"));

	}

	public void configureBindings() {
		// CANdle Statuses
		candle.setDefaultCommand(candle.titanCommand());

		// Gamepiece LED Status
		hasCoral.onTrue(candle.changeAnimationCommand(CANdleConstants.AnimationTypes.CORAL));
		hasAlgae.onTrue(candle.changeAnimationCommand(CANdleConstants.AnimationTypes.ALGAE));
		hasAlgae.and(hasCoral).onTrue(candle.changeAnimationCommand(CANdleConstants.AnimationTypes.BOTH));

		configureOperatorControls();
		configureDriverControls();
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}

}
