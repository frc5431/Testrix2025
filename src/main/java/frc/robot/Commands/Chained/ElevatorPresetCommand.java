package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsytems.Cleaner.CleanPivot;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.PresetPosition;

public class ElevatorPresetCommand extends Command {

    private PresetPosition position;
    private Elevator elevator;
    private ManipJoint manipJoint;
    private CleanPivot cleanPivot;

    public ElevatorPresetCommand(PresetPosition position, Elevator elevator, ManipJoint manipJoint, CleanPivot cleanPivot) {
        this.position = position;
        this.elevator = elevator;
        this.manipJoint = manipJoint;
        this.cleanPivot = cleanPivot;
        this.addRequirements(elevator, manipJoint, cleanPivot);
    }

    @Override
    public void initialize() {
        elevator.runEnum(position.getElevatorMode());
        manipJoint.runEnum(position.getJointMode());
        cleanPivot.runEnum(position.getPivotMode());
    }
    
}
