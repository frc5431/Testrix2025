package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;

public class ElevatorStowCommand extends SequentialCommandGroup {

	public ElevatorStowCommand(Elevator elevator, ManipJoint manipJoint) {

		switch (elevator.getPosition()) {
			// if higher than stow, dont get higher
			case EJECET:
			case CORALL4:
			case SAFESWING:
			case CORALL3:
			case CLEANL2:
			case CORALL2:
			case CORALL1:
			case STOW:
				addCommands(
						manipJoint.runManipJointCommand(ManipJointPositions.STOW),
						new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointConstants.stow,
								ManipJointConstants.error)),
						elevator.runElevatorCommand(ElevatorPositions.STOW));
				break;
			// if lower than stow, raise elevator first
			case FEED:
			default:
				addCommands(
						// rises to l4 so manip can safely move
						elevator.runElevatorCommand(ElevatorPositions.SAFESWING),
						new WaitUntilCommand(() -> elevator.getPositionSetpointGoal(ElevatorConstants.safeSwing,
								ElevatorConstants.error)),
						// manip runs to stow position only if the elevator is at the setpoint goal

						manipJoint.runManipJointCommand(ManipJointPositions.STOW),
						new WaitUntilCommand(() -> manipJoint.getPositionSetpointGoal(ManipJointConstants.stow,
								ManipJointConstants.error)),
						// since its sequential, this lowers once the manip is
						elevator.runElevatorCommand(ElevatorPositions.STOW));
				break;
		}

		addRequirements(elevator, manipJoint);
	}

}
