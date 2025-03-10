package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.TitanConditionalCommand;
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

        addCommands(
                new TitanConditionalCommand(new ElevatorStowCommand(elevator, manipJoint),
                        new ElevatorFeedCommand(elevator, manipJoint), manipulator.getBeambreakStatus()));

        addRequirements(elevator, manipJoint, manipulator);

    }

}
