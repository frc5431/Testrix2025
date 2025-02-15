package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;

public class EjectCoralCommand extends ParallelCommandGroup {

    public EjectCoralCommand(Intake intake, Manipulator manipulator) {
        addCommands(
                intake.runIntakeCommand(IntakeModes.OUTTAKE),
                manipulator.runManipulatorCommand(ManipulatorModes.REVERSE));
    }

}
