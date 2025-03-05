package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;

public class ElevatorFeedCommand extends SequentialCommandGroup{
    
    /**
	 * @param elevator
	 * @param manipJoint
	 */
	public ElevatorFeedCommand(Elevator elevator, ManipJoint manipJoint) {
		addCommands(
				manipJoint.runManipJointCommand(ManipJointPositions.FEED),
				// when prev commands finish (instantaly since its RunCommands)
				// sets elevator to stow angle only if the manipulator is near the stow angle
				// this ensures that the manip doesnt hit anything while the elevator goes down
				elevator.runElevatorCommand(ElevatorPositions.FEED).onlyIf(() -> manipJoint
						.getPositionSetpointGoal(ManipJointConstants.feed, ManipJointConstants.error))

		);
		addRequirements(elevator, manipJoint);
	}

}
