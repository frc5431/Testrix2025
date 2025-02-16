package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsytems.Cleaner.CleanPivot;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class ScoreCoralCommand extends SequentialCommandGroup {
    /**
	 * @apiNote CleanPivot Version
	 * @param elevator
	 * @param manipJoint
     * @param manipulator
     * @param cleanPivot
	 */
	public ScoreCoralCommand(Elevator elevator, ManipJoint manipJoint, Manipulator manipulator, CleanPivot cleanPivot) {
		addCommands(
				// Score
                manipulator.runManipulatorCommand(ManipulatorModes.SCORE).until(() -> !manipulator.getBeambreakStatus()),
                // Stow
				new ElevatorStowCommand(CleanPivotModes.STOW, elevator, manipJoint, cleanPivot)

		);
		addRequirements(elevator, manipulator, cleanPivot);
	}

	/**
	 * @apiNote NON CleanPivot Version
	 * @param elevator
	 * @param manipJoint
	 * @param manipulator
	 * @param cleanPivot
	 */
	public ScoreCoralCommand(Elevator elevator, ManipJoint manipJoint, Manipulator manipulator) {
		addCommands(
				// Score
				manipulator.runManipulatorCommand(ManipulatorModes.SCORE)
						.until(() -> !manipulator.getBeambreakStatus()),
				// Stow
				new ElevatorStowCommand(elevator, manipJoint)

		);
		addRequirements(elevator, manipulator);
	}
}
