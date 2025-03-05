package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class SmartScoreCommand extends SequentialCommandGroup {

    /**
     * If scoring L2/L3, auto stow
     * In cases when scoring L4, I dont wanna climb the reef
     * 
     * @param elevator
     * @param manipJoint
     * @param manipulator
     */
    public SmartScoreCommand(Elevator elevator, ManipJoint manipJoint, Manipulator manipulator) {

        addCommands(
                (manipJoint.getMode() == ManipJointPositions.SCOREL2
                        || manipJoint.getMode() == ManipJointPositions.SCOREL2)
                                ? new ScoreCoralCommand(elevator, manipJoint, manipulator)
                                : manipulator.runManipulatorCommand(ManipulatorModes.SCORE));

        addRequirements(elevator, manipJoint, manipulator);

    }

}
