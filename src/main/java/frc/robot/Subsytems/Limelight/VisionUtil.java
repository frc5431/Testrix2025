package frc.robot.Subsytems.Limelight;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Robot;
import frc.robot.Subsytems.Limelight.LimelightHelpers.VisionHelper;
import frc.robot.Subsytems.Limelight.LimelightHelpers.VisionHelper.PhysicalConfig;
import frc.robot.Util.Constants.VisionConstants;
import frc.team5431.titan.core.vision.Limelight;

public class VisionUtil {

    public static class VisionCommandConfig {

        public Limelight limelight;
        public int pipeline;
        public double p;
        public double maxOutput;
        public Distance distanceTolerance;
        public Angle angleTolerance;

        public Distance v_distanceSetpoint;
        public Distance h_distanceSetpoint;

        public void setLimeLight(Limelight limelight, int pipeline) {
            this.limelight = limelight;
            this.pipeline = pipeline;
        }

        public void setP(double p) {
            this.p = p;
        }

        public void setMaximumOutput(double maxOutput) {
            this.maxOutput = maxOutput;
        }

        public void setDistanceTolerance(Distance tolerance) {
            this.distanceTolerance = tolerance;
        }

        public void setAngleTolerance(Angle tolerance) {
            this.angleTolerance = tolerance;
        }

        public void setVerticalDistanceSetpoint(Distance tolerance) {
            this.v_distanceSetpoint = tolerance;
        }

        public void setHorizontalDistanceSetpoint(Distance tolerance) {
            this.h_distanceSetpoint = tolerance;
        }
    }

    public static final class VisionConfig {
        /* Limelight Configuration */

        public static final String LEFT_LL = "limelight-left";
        public static final PhysicalConfig LEFT_CONFIG = new PhysicalConfig()
                .withTranslation(VisionConstants.leftLLForwardOffset.in(Units.Meters), VisionConstants.leftLLRightOffset.in(Units.Meters), VisionConstants.leftLLUpOffset.in(Units.Meters))
                .withRotation(VisionConstants.leftLLRollOffset.in(Units.Degrees), VisionConstants.leftLLPitchOffset.in(Units.Degrees), VisionConstants.leftLLYawOffset.in(Units.Degrees));

        public static final String RIGHT_LL = "limelight-right";
        public static final PhysicalConfig RIGHT_CONFIG = new PhysicalConfig()
                .withTranslation(VisionConstants.rightLLForwardOffset.in(Units.Meters), VisionConstants.rightLLRightOffset.in(Units.Meters), VisionConstants.rightLLUpOffset.in(Units.Meters))
                .withRotation(VisionConstants.rightLLRollOffset.in(Units.Degrees), VisionConstants.rightLLPitchOffset.in(Units.Degrees), VisionConstants.rightLLYawOffset.in(Units.Degrees));


        /* Pipeline configs */
        public static final int leftTagPipeline = 0;
        public static final int rightTagPipeline = 0;

        /* Pose Estimation Constants */
        public static final double VISION_REJECT_DISTANCE = 1.8; // 2.3;

        // Increase these numbers to trust global measurements from vision less.
        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix = VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y,
                VISION_STD_DEV_THETA);

        /* Vision Command Configs */

    }

    /* Limelights */
    public final VisionHelper leftLL = new VisionHelper(
            VisionConfig.LEFT_LL,
             VisionConfig.leftTagPipeline,
              VisionConfig.LEFT_CONFIG);
    public final VisionHelper rightLL = new VisionHelper(
            VisionConfig.RIGHT_LL,
            VisionConfig.rightTagPipeline,
            VisionConfig.RIGHT_CONFIG);
    
    public final VisionHelper[] allLimelights = {leftLL, rightLL};
    public final VisionHelper[] poseLimelights = {
            leftLL, rightLL
    };

    public static class LimelightLogger {                                           
        private final Limelight limelight;
        private String name;

        public LimelightLogger(String name, Limelight limelight) {
            this.limelight = limelight;
            this.name = name;
        }

        @AutoLogOutput(key = "Vision/{name}/ConnectionStatus")
        public boolean getCameraConnection() {
            return limelight.isCameraConnected();
        }

        @AutoLogOutput(key = "Vision/{name}/Integrating")
        public boolean getIntegratingStatus() {
            return limelight.isIntegrating;
        }

        @AutoLogOutput(key = "Vision/{name}/LogStatus")
        public String getLogStatus() {
            return limelight.logStatus;
        }

        @AutoLogOutput(key = "Vision/{name}/TagStatus")
        public String getTagStatus() {
            return limelight.tagStatus;
        }

        @AutoLogOutput(key = "Vision/{name}/Pose")
        public Pose2d getPose() {
            return limelight.getRawPose3d().toPose2d();
        }

        @AutoLogOutput(key = "Vision/{name}/MegaPose")
        public Pose2d getMegaPose() {
            return limelight.getMegaPose2d();
        }

        @AutoLogOutput(key = "Vision/{name}/PoseX")
        public double getPoseX() {
            return getPose().getX();
        }

        @AutoLogOutput(key = "Vision/{name}/PoseY")
        public double getPoseY() {
            return getPose().getY();
        }

        @AutoLogOutput(key = "Vision/{name}/TagCount")
        public double getTagCount() {
            return limelight.getTagCountInView();
        }

        @AutoLogOutput(key = "Vision/{name}/TargetSize")
        public double getTargetSize() {
            return limelight.getTargetSize();
        }
    }


}
