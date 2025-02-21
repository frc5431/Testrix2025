package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.PresetPosition;

public class ElevatorPresetCommand extends Command {

    private PresetPosition position;
    private Elevator elevator;
    private ManipJoint manipJoint;

    /**
     * @param position
     * @param elevator
     * @param manipJoint
     */
    public ElevatorPresetCommand(PresetPosition position, Elevator elevator, ManipJoint manipJoint) {
        this.position = position;
        this.elevator = elevator;
        this.manipJoint = manipJoint;
        this.addRequirements(elevator, manipJoint);
    }

    @Override
    public void initialize() {
        elevator.runEnum(position.getElevatorMode());
        manipJoint.runEnum(position.getJointMode());
    }
    
}
