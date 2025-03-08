package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;

public class SmartStowCommand extends SequentialCommandGroup {

    /**
     * S
     * @param elevator
     * @param manipJoint
     * @param manipulator
     */
    public SmartStowCommand(Elevator elevator, ManipJoint manipJoint, Manipulator manipulator) {

        addCommands((manipulator.getBeambreakStatus()) ? new ElevatorStowCommand(elevator, manipJoint)
                : new ElevatorFeedCommand(elevator, manipJoint));

        addRequirements(elevator, manipJoint, manipulator);

    }

}
