package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.PresetPosition;

public class ElevatorPresetCommand extends SequentialCommandGroup {

	public ElevatorPresetCommand(PresetPosition position, Elevator elevator, ManipJoint manipJoint) {

		switch (manipJoint.getMode()) {
			case FEED:
			case PREEFEED:
				addCommands(new ElevatorStowCommand(elevator, manipJoint),
						new ElevatorPresetCommand(position, elevator, manipJoint));
				break;
			default:
				addCommands(manipJoint.runManipJointCommand(position.getJointMode()).alongWith(
						elevator.runElevatorCommand(position.getElevatorMode())));
				break;

		}
		addRequirements(elevator, manipJoint);
	}

}
