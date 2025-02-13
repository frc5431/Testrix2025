package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Cleaner.CleanPivot;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotModes;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;

public class ElevatorStowCommand extends SequentialCommandGroup {

    /**
     * @param mode
     * @param elevator 
     * @param manipJoint
     * @param cleanPivot
     */
    public ElevatorStowCommand(CleanPivotModes mode, Elevator elevator, ManipJoint manipJoint, CleanPivot cleanPivot) {
        addCommands(
                //runs (cleaner) pivot command in junction with running manipJoint to stow
                cleanPivot.runCleanerPivotCommand(mode)
                        .alongWith(manipJoint.runCleanerPivotCommand(ManipJointPositions.STOW),
                        //when prev commands finish (instantaly since it they are RunCommands)
                        //sets elevator to stow angle only if the manipulator is near the stow angle
                        //this ensures that the manip doesnt hit anything while the elevator goes down
                                elevator.runElevatorCommand(ElevatorPositions.STOW).onlyIf(() -> manipJoint
                                        .getAngleSetpointGoal(ManipJointConstants.stow, ManipJointConstants.error))

                        ));
    }

}
