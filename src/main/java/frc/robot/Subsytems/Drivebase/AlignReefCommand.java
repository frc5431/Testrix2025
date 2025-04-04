/*
package frc.robot.Subsytems.Drivebase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Systems;
import frc.robot.Subsytems.CANdle.TitanCANdle;
import frc.robot.Subsytems.Limelight.Vision;
import frc.robot.Util.Constants.VisionConstants;

public class AlignReefCommand extends SequentialCommandGroup {
    private Drivebase drivebase = Systems.getDrivebase();
    private Vision vision = Systems.getVision();
    private TitanCANdle candle = Systems.getTitanCANdle();

    public AlignReefCommand(boolean rightTrue) {

        if (vision.OnlyIfNullChecker()) {
            addCommands(
                    drivebase.faceTargetCommand(vision.getBestLimelight().getMegaPose2d().getRotation()),
                    drivebase
                            .driveRobotCentric(
                                    rightTrue ? VisionConstants.alignXSpeed : VisionConstants.alignXSpeed.times(-1)),
                                    new WaitUntilCommand(vision.getPipeAlignDist(rightTrue)),
                    drivebase.driveRobotCentric(VisionConstants.alignYSpeed).until(() -> vision.getPipeScoreDist()));
        } else {
            addCommands();
        }

        // alongWith(candle.changeAnimation(AnimationTypes.BLINK_RED));

        // andThen(candle.changeAnimationCommand(AnimationTypes.FLASHING_GREEN).withTimeout(10));

        addRequirements(drivebase, vision);// , candle);
    }

    public AlignReefCommand() {
        if (vision.OnlyIfNullChecker()) {
            addCommands(
                    drivebase.faceTargetCommand(vision.getBestLimelight().getMegaPose2d().getRotation()),
                    drivebase.driveRobotCentric(
                            vision.leftOfTag() ? VisionConstants.alignXSpeed : VisionConstants.alignXSpeed.times(-1))
                            .until(() -> vision.isCentered()));

            // drivebase.driveRobotCenteric(VisionConstants.alignYSpeed).until(() ->
            // vision.getCenterScoreDistance()));
        } else {
            addCommands();
        }

        // alongWith(candle.changeAnimationCommand(AnimationTypes.BLINK_BLUE));

        // andThen(candle.changeAnimationCommand(AnimationTypes.FLASHING_GREEN).withTimeout(10));

        addRequirements(drivebase, vision);// , candle);
    }
}
*/