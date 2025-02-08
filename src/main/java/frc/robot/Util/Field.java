package frc.robot.Util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Field {
    public static final Distance fieldLength = Units.Inches.of(690.876);
    private static final Distance halfLength = fieldLength.div(2);
    public static final Distance fieldWidth = Units.Inches.of(317);
    private static final Distance halfWidth = fieldWidth.div(2);

    public static final Distance startingLineX =
            Units.Inch.of(299.438); // Measured from the inside of starting line

    public static class Processor {
        public static final Pose2d centerFace =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage =
                new Translation2d(Units.Inches.of(345.428), Units.Inches.of(286.779));
        public static final Translation2d middleCage =
                new Translation2d(Units.Inches.of(345.428), Units.Inches.of(242.855));
        public static final Translation2d closeCage =
                new Translation2d(Units.Inches.of(345.428), Units.Inches.of(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.Inches.of(3.125);
        public static final double shallowHeight = Units.Inches.of(30.125);
    }

    public static class CoralStation {
        public static final Pose2d leftCenterFace =
                new Pose2d(
                        Units.Inches.of(33.526),
                        Units.Inches.of(291.176),
                        Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace =
                new Pose2d(
                        Units.Inches.of(33.526),
                        Units.Inches.of(25.824),
                        Rotation2d.fromDegrees(144.011 - 90));
    }

    public static class Reef {
        public static final Translation2d center =
                new Translation2d(Units.Inches.of(176.746), Units.Inches.of(158.501));
        public static final double faceToZoneLine =
                Units.Inches.of(12); // Side of the reef to the inside of the reef zone line

        public static final Pose2d[] centerFaces =
                new Pose2d[6]; // Starting facing the driver station in clockwise order
        public static final List<Map<ReefHeight, Pose3d>> branchPositions =
                new ArrayList<>(); // Starting at the right branch facing the driver station in
        // clockwise

        static {
            // Initialize faces
            centerFaces[0] =
                    new Pose2d(
                            Units.Inches.of(144.003),
                            Units.Inches.of(158.500),
                            Rotation2d.fromDegrees(180));
            centerFaces[1] =
                    new Pose2d(
                            Units.Inches.of(160.373),
                            Units.Inches.of(186.857),
                            Rotation2d.fromDegrees(120));
            centerFaces[2] =
                    new Pose2d(
                            Units.Inches.of(193.116),
                            Units.Inches.of(186.858),
                            Rotation2d.fromDegrees(60));
            centerFaces[3] =
                    new Pose2d(
                            Units.Inches.of(209.489),
                            Units.Inches.of(158.502),
                            Rotation2d.fromDegrees(0));
            centerFaces[4] =
                    new Pose2d(
                            Units.Inches.of(193.118),
                            Units.Inches.of(130.145),
                            Rotation2d.fromDegrees(-60));
            centerFaces[5] =
                    new Pose2d(
                            Units.Inches.of(160.375),
                            Units.Inches.of(130.144),
                            Rotation2d.fromDegrees(-120));

            // Initialize branch positions
            for (int face = 0; face < 6; face++) {
                Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
                Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
                for (var level : ReefHeight.values()) {
                    Pose2d poseDirection =
                            new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                    Distance adjustX = Units.Inches.of(30.738);
                    Distance adjustY = Units.Inches.of(6.469);

                    fillRight.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY,
                                                                    new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY,
                                                                    new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                    fillLeft.put(
                            level,
                            new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY.div(-1),
                                                                    new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(
                                                            new Transform2d(
                                                                    adjustX,
                                                                    adjustY.div(-1),
                                                                    new Rotation2d()))
                                                    .getY(),
                                            level.height),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitch),
                                            poseDirection.getRotation().getRadians())));
                }
                branchPositions.add((face * 2) + 1, fillRight);
                branchPositions.add((face * 2) + 2, fillLeft);
            }
        }
    }

    public static class StagingPositions {
        // Measured from the center of the ice cream
        public static final Pose2d leftIceCream =
                new Pose2d(Units.Inches.of(48), Units.Inches.of(230.5), new Rotation2d());
        public static final Pose2d middleIceCream =
                new Pose2d(Units.Inches.of(48), Units.Inches.of(158.5), new Rotation2d());
        public static final Pose2d rightIceCream =
                new Pose2d(Units.Inches.of(48), Units.Inches.of(86.5), new Rotation2d());
    }

    public enum ReefHeight {
        L4(Units.Inches.of(72), Units.Inches.of(-90)),
        L3(Units.Inches.of(47.625), Units.Inches.of(-35)),
        L2(Units.Inches.of(31.875), Units.Inches.of(-35)),
        L1(Units.Inches.of(18), Units.Inches.of(0));

        ReefHeight(Distance height, Distance pitch) {
                    this.height = height;
                    this.pitch = pitch; // in degrees
        }

        public final Distance height;
        public final Distance pitch;
    }

    private static final double aprilTagWidth = Units.inchesToMeters(6.50);

    /** Returns {@code true} if the robot is on the blue alliance. */
    public static boolean isBlue() {
        return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
    }

    /** Returns {@code true} if the robot is on the red alliance. */
    public static boolean isRed() {
        return !isBlue();
    }

    public static final Trigger red = new Trigger(() -> isRed());
    public static final Trigger blue = new Trigger(() -> isBlue());

    // Flip the angle if we are blue, as we are setting things for a red driver station angle
    // This flips the left and right side for aiming purposes
    public static double flipAimAngleIfBlue(double redAngleDegs) {
        if (Field.isBlue()) {
            return 180 - redAngleDegs;
        }
        return redAngleDegs;
    }

    // This flips the true angle of the robot if we are blue
    public static double flipTrueAngleIfBlue(double redAngleDegs) {
        if (Field.isBlue()) {
            return (180 + redAngleDegs) % 360;
        }
        return redAngleDegs;
    }

    public static double flipTrueAngleIfRed(double blueAngleDegs) {
        if (Field.isRed()) {
            return (180 + blueAngleDegs) % 360;
        }
        return blueAngleDegs;
    }

    public static Rotation2d flipAngleIfRed(Rotation2d blue) {
        if (Field.isRed()) {
            return new Rotation2d(-blue.getCos(), blue.getSin());
        } else {
            return blue;
        }
    }

    public static Pose2d flipXifRed(Pose2d blue) {
        return new Pose2d(
                flipXifRed(blue.getX()), blue.getTranslation().getY(), blue.getRotation());
    }

    public static Translation2d flipXifRed(Translation2d blue) {
        return new Translation2d(flipXifRed(blue.getX()), blue.getY());
    }

    public static Translation3d flipXifRed(Translation3d blue) {
        return new Translation3d(flipXifRed(blue.getX()), blue.getY(), blue.getZ());
    }

    // If we are red flip the x pose to the other side of the field
    public static double flipXifRed(double xCoordinate) {
        if (Field.isRed()) {
            return Field.fieldLength - xCoordinate;
        }
        return xCoordinate;
    }

    // If we are red flip the y pose to the other side of the field
    public static double flipYifRed(double yCoordinate) {
        if (Field.isRed()) {
            return Field.fieldWidth - yCoordinate;
        }
        return yCoordinate;
    }

    public static boolean poseOutOfField(Pose2d pose2D) {
        double x = pose2D.getX();
        double y = pose2D.getY();
        return (x <= 0 || x >= fieldLength) || (y <= 0 || y >= fieldWidth);
    }

    public static boolean poseOutOfField(Pose3d pose3D) {
        return poseOutOfField(pose3D.toPose2d());
    }
}