package frc.robot.Subsytems.Drivebase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Systems;
import frc.robot.Subsytems.CANdle.TitanCANdle;
import frc.robot.Subsytems.Limelight.Vision;
import frc.robot.Util.Constants.CANdleConstants.AnimationTypes;
import frc.robot.Util.Constants.VisionConstants;
import frc.robot.Util.Field;

public class AlignReefCommand extends SequentialCommandGroup {
    private Drivebase drivebase = Systems.getDrivebase();
    private Vision vision = Systems.getVision();
    private TitanCANdle candle = Systems.getTitanCANdle();
    private AprilTagFieldLayout aprilTagFieldLayout = Systems.getApriltagLayout();
//TODO ADD NULL CHECKING
    public AlignReefCommand(boolean rightTrue) {
        addCommands(
                drivebase.faceTargetCommand(aprilTagFieldLayout
                        .getTagPose((int) vision.getBestLimelight().getClosestTagID()).get().getRotation()
                        .toRotation2d()),
                drivebase.driveRobotCenteric(rightTrue ? VisionConstants.alignXSpeed : VisionConstants.alignXSpeed.times(-1
                ))
                        .until(() -> vision.getPipeAlignDist(rightTrue)),
                drivebase.driveRobotCenteric(VisionConstants.alignYSpeed).until(() -> vision.getPipeScoreDist()));

        alongWith(candle.changeAnimationCommand(AnimationTypes.BLINK_RED));

        andThen(candle.changeAnimationCommand(AnimationTypes.FLASHING_GREEN).withTimeout(10));

        onlyIf(() -> (Field.isRedTag(vision.getBestLimelight().getClosestTagID()) == Field.isRed())
                && Field.isReef(vision.getBestLimelight().getClosestTagID()));

        addRequirements(drivebase, vision, candle);
    }

    public AlignReefCommand() {
        addCommands(
                drivebase.faceTargetCommand(aprilTagFieldLayout
                        .getTagPose((int) vision.getBestLimelight().getClosestTagID()).get().getRotation()
                        .toRotation2d()),
                drivebase.driveRobotCenteric(vision.leftOfTag() ? VisionConstants.alignXSpeed :  VisionConstants.alignXSpeed.times(-1)).until(() -> vision.isCentered()));
                // drivebase.driveRobotCenteric(VisionConstants.alignYSpeed).until(() -> vision.getCenterScoreDistance()));

        alongWith(candle.changeAnimationCommand(AnimationTypes.BLINK_BLUE));

        andThen(candle.changeAnimationCommand(AnimationTypes.FLASHING_GREEN).withTimeout(10));

        onlyIf(() -> vision.OnlyIfNullChecker());

        addRequirements(drivebase, vision, candle);
    }
}

