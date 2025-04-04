package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Intake.IntakePivot;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class IntakeCoralCommand extends SequentialCommandGroup {

        /**
         * @param intake
         * @param feeder
         * @param manipulator
         */
        public IntakeCoralCommand(Intake intake, IntakePivot intakePivot, Manipulator manipulator, Elevator elevator,
                        ManipJoint manipJoint) {
                addCommands(
                                new ParallelCommandGroup(
                                                intakePivot.runIntakePivotCommand(IntakePivotModes.DEPLOY),
                                                new ElevatorFeedCommand(elevator, manipJoint),
                                                manipulator.runManipulatorCommand(ManipulatorModes.FEED))
                                                                .until(() -> manipulator.hasCoral()),
                                manipulator.runManipulatorCommand(ManipulatorModes.FEED).withTimeout(0.1),
                                new ElevatorStowCommand(elevator, manipJoint)

                );

                addRequirements(intake, intakePivot, manipulator, manipJoint, elevator);

        }
}
