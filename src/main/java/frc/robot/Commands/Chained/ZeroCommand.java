package frc.robot.Commands.Chained;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsytems.Elevator.Elevator;

public class ZeroCommand extends SequentialCommandGroup {

    /**
     * fuck this. why does this work so well. fuck
     * @param elevator
     */
    public ZeroCommand(Elevator elevator) {
            
        addCommands(
                new InstantCommand(() -> elevator.setPercentOutput(-.3)),
                new WaitCommand(0.5),
                new InstantCommand(() -> elevator.setPercentOutput(0.0)),
                elevator.zeroElevatorCommand());
        addRequirements(elevator);
    }
}
