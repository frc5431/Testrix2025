// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsytems.Cleaner.CleanPivot;
import frc.robot.Subsytems.Cleaner.Cleaner;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.*;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.team5431.titan.core.joysticks.TitanController;

public class RobotContainer {

	private final Systems systems = new Systems();

	private final Intake intake = systems.getIntake();
	private final Cleaner cleaner = systems.getCleaner();
	private final Elevator elevator = systems.getElevator();
	private final CleanPivot cleanPivot = systems.getCleanPivot();
	private final ManipJoint manipJoint = systems.getManipJoint();
	private final Manipulator manipulator = systems.getManipulator();

	private TitanController driver = new TitanController(ControllerConstants.driverPort, ControllerConstants.deadzone);
	private TitanController operator = new TitanController(ControllerConstants.operatorPort,
			ControllerConstants.deadzone);

	// Controls

	// Driver Controls

	// Operator Controls

	// Intake Controls
	private Trigger intakeCoral = operator.a();

	public RobotContainer() {
		configureBindings();
	}

	public void periodicSubsystems() {
		intake.periodic();
		elevator.periodic();
		cleaner.periodic();
		cleanPivot.periodic();
		manipJoint.periodic();
		manipulator.periodic();
	}

	public void periodic() {
		periodicSubsystems();
		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

	}

	private void configureDriverControls() {

	}

	private void configureOperatorControls() {

		// Intake Controls
		intakeCoral.whileTrue(intake.runIntakeCommand(IntakeModes.INTAKE));

	}

	public void configureBindings() {
		configureOperatorControls();

	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}

}
