package frc.robot.Subsytems.Drivebase;

import frc.robot.Util.Field;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Systems;
import frc.robot.Subsytems.Limelight.Vision;

public class AlignReefCommand extends SequentialCommandGroup {
    private Drivebase drivebase = Systems.getDrivebase();
    private Vision vision = Systems.getVision();
    private AprilTagFieldLayout aprilTagFieldLayout = Systems.getApriltagLayout();

    public AlignReefCommand(Boolean rightTrue) {
        addCommands(
                drivebase.faceTargetCommand(aprilTagFieldLayout
                        .getTagPose((int) vision.getBestLimelight().getClosestTagID()).get().getRotation()
                        .toRotation2d()));


        onlyIf(() -> Field.isRedTag(vision.getBestLimelight().getClosestTagID()) == Field.isRed()
                && Field.isReef(vision.getBestLimelight().getClosestTagID()));
        addRequirements(drivebase, vision);
    }
}
