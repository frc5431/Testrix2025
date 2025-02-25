package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Chained.ElevatorFeedCommand;
import frc.robot.Commands.Chained.ElevatorStowCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Intake.IntakePivot;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class AutoIntakeCoralCommand extends SequentialCommandGroup {

    /**
     * @param intake
     * @param feeder
     * @param manipulator
     */
    public AutoIntakeCoralCommand(Intake intake, IntakePivot intakePivot, Manipulator manipulator, Elevator elevator,
            ManipJoint manipJoint) {
        addCommands(
                new ParallelCommandGroup(
                        intakePivot.runIntakePivotCommand(IntakePivotModes.DEPLOY),
                        new ElevatorFeedCommand(elevator, manipJoint),
                        intake.runIntakeCommand(IntakeModes.INTAKE),
                        manipulator.runManipulatorCommand(ManipulatorModes.FEED))
                                .until(() -> manipulator.getBeambreakStatus()),
                new ParallelCommandGroup(
                        manipulator.runManipulatorCommand(ManipulatorModes.FEED).withTimeout(0.1)
                                .andThen(manipulator.runManipulatorCommand(ManipulatorModes.IDLE)),
                        intakePivot.runIntakePivotCommand(IntakePivotModes.STOW)),
                new ElevatorStowCommand(elevator, manipJoint)

        );

        addRequirements(intake, intakePivot, manipulator, manipJoint, elevator);

    }
}
