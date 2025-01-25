// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Util.Constants.*;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.team5431.titan.core.joysticks.TitanController;

public class RobotContainer {

  private System systems = new System();
  private Intake intake;

  private TitanController driver = new TitanController(ControllerConstant.driverPort, ControllerConstant.deadzone);
  private TitanController operator = new TitanController(ControllerConstant.operatorPort, ControllerConstant.deadzone);

  // Controls

  // Driver Controls

  // Operator Controls

  // Intake Controls
  private Trigger intakeCoral = operator.a();

  public RobotContainer() {

    intake = systems.getIntake();

    configureBindings();
  }

  public void periodic() {

  }

  private void configureBindings() {

    // Operator Controls

    // Intake Controls
    intakeCoral.whileTrue(intake.runIntakeCommand(IntakeModes.INTAKE));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
