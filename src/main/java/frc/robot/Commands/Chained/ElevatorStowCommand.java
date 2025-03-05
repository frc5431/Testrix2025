package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;

public class ElevatorStowCommand extends SequentialCommandGroup {

	public ElevatorStowCommand(Elevator elevator, ManipJoint manipJoint) {
		addCommands(
				//rises to l2 so manip can safely move
				elevator.runElevatorCommand(ElevatorPositions.CORALL2),
				//manip runs to stow position only if the elevator is at the setpoint goal
				manipJoint.runManipJointCommand(ManipJointPositions.STOW).onlyIf(
					() -> elevator.getPositionSetpointGoal(ElevatorConstants.coralL2, ElevatorConstants.error)),
				//since its sequential, this lowers once the manip is 
				elevator.runElevatorCommand(ElevatorPositions.FEED)
		);

		addRequirements(elevator, manipJoint);
	}

}
