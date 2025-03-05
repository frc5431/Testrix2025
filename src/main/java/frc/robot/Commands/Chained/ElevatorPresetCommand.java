package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import frc.robot.Util.PresetPosition;

public class ElevatorPresetCommand extends ParallelCommandGroup {

	public ElevatorPresetCommand(PresetPosition position, Elevator elevator, ManipJoint manipJoint) {
		addCommands(
            manipJoint.runManipJointCommand(position.getJointMode()),
            elevator.runElevatorCommand(position.getElevatorMode())
				
		);
        // saftey protections
        onlyIf(() -> manipJoint.getMode() != ManipJointPositions.FEED);
		addRequirements(elevator, manipJoint);
	}

}

