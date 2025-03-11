package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;

public class SmartStowCommand extends SequentialCommandGroup {

    /**
     * S
     * 
     * @param elevator
     * @param manipJoint
     * @param manipulator
     */
    public SmartStowCommand(Elevator elevator, ManipJoint manipJoint, Manipulator manipulator) {
        if (manipulator.hasCoral() == true) {
            addCommands(
                new ElevatorFeedCommand(elevator, manipJoint)
            );
        } else if(manipulator.hasCoral() == false) {
            addCommands(
                new ElevatorStowCommand(elevator, manipJoint)
            );
        } else {
            DriverStation.reportError("SMART STOW NOT RECIVING BEAMBREAK STATUS", true);
        }

        addRequirements(elevator, manipJoint);

    }

}
