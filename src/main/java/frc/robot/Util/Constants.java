package frc.robot.Util;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotModes;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;

public final class Constants {

    public static final String canbus = "Omnivore";

    public static class ControllerConstants {

        public enum ControlStates {
            ALGAE,
            CORAL,
        }

        public static final int driverPort = 0;
        public static final int operatorPort = 1;

        public static final double deadzone = 0.15;

    }

    public static class opConst {

        public static final PresetPosition StowPosition = new PresetPosition(
                ElevatorPositions.STOW, ManipJointPositions.STOW, CleanPivotModes.STOW);

        public static final PresetPosition FeedCoralPosition = new PresetPosition(
                ElevatorPositions.FEED, ManipJointPositions.STOW, CleanPivotModes.STOW);

        public static final PresetPosition ScoreL1Position = new PresetPosition(ElevatorPositions.CORALL1,
                ManipJointPositions.SCOREL1);

        public static final PresetPosition ScoreL2Position = new PresetPosition(ElevatorPositions.CORALL2,
                ManipJointPositions.SCOREL2, CleanPivotModes.L2);

        public static final PresetPosition ScoreL3Position = new PresetPosition(ElevatorPositions.CORALL3,
                ManipJointPositions.SCOREL3, CleanPivotModes.L3);

        public static final PresetPosition ScoreL4Position = new PresetPosition(ElevatorPositions.CORALL4,
                ManipJointPositions.SCOREL4, CleanPivotModes.STOW);

    }

    public static class IntakeConstants {

        public enum IntakeStates {
            IDLE,
            INTAKING,
            FEEDING,
            OUTTAKING,
            STUCK,
        }

        public static final boolean attached = true;
        public static final boolean isInverted = false;
        public static final int id = 21;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(0);
        public static final Current stallLimit = Units.Amps.of(0);
        public static final Angle offset = Units.Rotation.of(0);
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;

        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final AngularVelocity intakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum IntakeModes {
            IDLE(idleSpeed),
            INTAKE(intakeSpeed),
            FEED(feedSpeed),
            OUTTAKE(outtakeSpeed);

            public AngularVelocity speed;

            IntakeModes(AngularVelocity speed) {
                this.speed = speed;
            }

        }

    }

    public static class CleanerConstants {

        public enum CleanerStates {
            IDLE,
            INTAKING,
            OUTTAKING
        }

        public static final boolean attached = true;
        public static final int id = 26;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(0);
        public static final Current stallLimit = Units.Amps.of(0);
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = false;
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;
        public static final double maxIAccum = 0;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);
        public static final AngularVelocity intakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);

        public enum CleanerModes {
            IDLE(idleSpeed),
            INTAKE(intakeSpeed),
            OUTTAKE(outtakeSpeed);

            public AngularVelocity speed;

            CleanerModes(AngularVelocity speed) {
                this.speed = speed;
            }

        }

    }

    public static class CleanPivotConstants {

        public enum CleanPivotStates {
            STOW,
            L2,
            L3,
            NET,
        }

        public static final int id = 25;
        public static final boolean attached = true;
        public static final Angle softLimitForwardMax = Units.Rotation.of(0);
        public static final boolean softLimitEnabled = true;
        public static final Angle intakeAngle = Units.Rotation.of(0);
        public static final Angle softLimitReverseMax = Units.Rotation.of(0);
        public static final Angle stowAngle = Units.Rotation.of(0);
        public static final Angle l2Angle = Units.Rotation.of(0);
        public static final Angle l3Angle = Units.Rotation.of(0);
        public static final Angle netAngle = Units.Rotation.of(0);
        public static final boolean isInverted = false;
        public static final Angle zeroOffset = Units.Rotation.of(0);
        public static final FeedbackSensor feedbackSensor = FeedbackSensor.kAbsoluteEncoder;
        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;

        public enum CleanPivotModes {
            INTAKE(intakeAngle),
            STOW(stowAngle),
            L2(l2Angle),
            L3(l3Angle),
            NET(netAngle);

            public Angle angle;

            CleanPivotModes(Angle angle) {
                this.angle = angle;
            }

        }

    }

    public static class ManipulatorConstants {

        public enum ManipulatorStates {
            IDLE,
            LOCKED,
            INTAKING,
            OUTTAKING,
        }

        public static final boolean attached = true;
        public static final int channel = 0;
        public static final int id = 24;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(0);
        public static final Current stallLimit = Units.Amps.of(0);
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = false;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final AngularVelocity scoreSpeed = Units.RPM.of(0);
        public static final AngularVelocity reverseSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ManipulatorModes {
            IDLE(idleSpeed),
            SCORE(scoreSpeed),
            FEED(feedSpeed),
            REVERSE(reverseSpeed);

            public AngularVelocity speed;

            ManipulatorModes(AngularVelocity speed) {
                this.speed = speed;
            }

        }

    }

    public static class ElevatorConstants {

        public enum ElevatorStates {
            STOWED,
            FEED,
            PROCESSOR,
            L1,
            L2,
            L3,
            L4,
            NET,
        }

        public static final boolean attached = true;
        public static final boolean leaderInvert = false;
        public static final boolean follwerInvert = true;
        public static final boolean gravityType = false;
        public static final boolean breakType = true;
        public static final boolean useFMaxRotation = true;
        public static final boolean useRMaxRotation = true;
        public static final boolean canRangeAttached = true;

        public static final int leftId = 14;
        public static final int rightId = 15;
        public static final int canCoderId = 16;
        public static final int canRangeId = 17;
        public static final double gearRatio = 1 / 1;
        public static final Current forwardCurrentLimit = Units.Amps.of(0);
        public static final Current reverseCurrentLimit = Units.Amps.of(0);

        public static final Current stallLimit = Units.Amps.of(0);
        public static final Angle offset = Units.Rotation.of(0);
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;
        public static final Angle maxReverseRotation = Units.Rotation.of(0);
        public static final Angle maxFowardRotation = Units.Rotation.of(0);
        public static final Angle rotationOffset = Units.Rotation.of(0);

        public static final FeedbackSensorSourceValue feedbackSensor = FeedbackSensorSourceValue.FusedCANcoder;

        public static final double s = 0;
        public static final double v = 0;
        public static final double a = 0;
        public static final double g = 0;
        public static final double ff = 0;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final Angle stow = Units.Rotation.of(0);
        public static final Angle feed = Units.Rotation.of(0);
        public static final Angle algaeProcessor = Units.Rotation.of(0);
        public static final Angle coralL1 = Units.Rotation.of(0);
        public static final Angle coralL2 = Units.Rotation.of(0);
        public static final Angle algaeL2 = Units.Rotation.of(0);
        public static final Angle coralL3 = Units.Rotation.of(0);
        public static final Angle algaeL3 = Units.Rotation.of(0);
        public static final Angle coralL4 = Units.Rotation.of(0);
        public static final Angle coralStation = Units.Rotation.of(0);
        public static final Angle net = Units.Rotation.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ElevatorPositions {
            STOW(stow),
            FEED(feed),
            PROCESSOR(algaeProcessor),
            CORALL1(coralL1),
            CORALL2(coralL2),
            ALGAEL2(algaeL2),
            CORALL3(coralL3),
            ALGAEL3(algaeL3),
            CORALL4(coralL4),
            NET(net);

            public Angle rotation;

            ElevatorPositions(Angle rotation) {
                this.rotation = rotation;
            }

        }

    }

    public static class ManipJointConstants {

        public enum ManipJointStates {
            STOWED,
            FEED,
            HUMAN,
            L1,
            L2,
            L3,
            L4
        }

        public static final boolean attached = true;
        public static final int id = 23;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(30);
        public static final Current stallLimit = Units.Amps.of(50);
        public static final IdleMode idleMode = IdleMode.kBrake;
        public static final boolean isInverted = false;
        public static final Angle offset = Units.Rotation.of(0);
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final Angle stow = Units.Rotations.of(0);
        public static final Angle feed = Units.Rotations.of(0);
        public static final Angle scoreL1 = Units.Rotations.of(0);
        public static final Angle scoreL2 = Units.Rotations.of(0);
        public static final Angle scoreL3 = Units.Rotations.of(0);
        public static final Angle scoreL4 = Units.Rotations.of(0);
        public static final Angle error = Units.Rotations.of(0);

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ManipJointPositions {
            STOW(stow),
            FEED(feed),
            SCOREL1(scoreL1),
            SCOREL2(scoreL2),
            SCOREL3(scoreL3),
            SCOREL4(scoreL4);

            public Angle position;

            ManipJointPositions(Angle position) {
                this.position = position;
            }

        }

    }
     public static class FeederConstants {

        public enum FeederStates {
            IDLE,
            STUCK,
            REVERSE,
            FEEDING,
        }

        public static final boolean attached = true;
        public static final boolean isInverted = false;
        public static final int id = 18;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(0);
        public static final Current stallLimit = Units.Amps.of(0);
        public static final Angle offset = Units.Rotation.of(0);
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;

        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final AngularVelocity reverseSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum FeederModes {
            IDLE(idleSpeed),
            REVERSE(reverseSpeed),
            FEED(feedSpeed);

            public AngularVelocity speed;

            FeederModes(AngularVelocity speed) {
                this.speed = speed;
            }

        }
     }

    public static class ClimberConstants {

        public enum ClimberStates {
            STOW,
            ALIGN,
            CLIMB
        }

        public static final boolean attached = true;
        public static final boolean isInverted = false;
        public static final int id = 27;
        public static final double gearRatio = 1 / 1;
        public static final Current supplyLimit = Units.Amps.of(0);
        public static final Current stallLimit = Units.Amps.of(0);
        public static final Angle offset = Units.Rotation.of(0);
        public static final double maxForwardOutput = 0;
        public static final double maxReverseOutput = 0;

        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final FeedbackSensor sensorType = FeedbackSensor.kPrimaryEncoder;
        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;
        public static final double maxIAccum = 0;

        public static final Angle stow_angle = Units.Rotation.of(0);
        public static final Angle align_angle = Units.Rotation.of(0);
        public static final Angle climb_angle = Units.Rotation.of(0);
        public static final Angle error = Units.Rotation.of(0);

        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ClimberModes {
            STOW(stow_angle),
            ALIGN(align_angle),
            CLIMB(climb_angle);

            public Angle angle;

            ClimberModes(Angle angle) {
                this.angle = angle;
            }

        }

    }

    public static class IntakePivotConstants {

        public enum IntakePivotStates {
            STOW,
            INTAKING,
        }

        public static final int id = 5555555;
        public static final boolean attached = true;
        public static final Angle softLimitForwardMax = Units.Rotation.of(0);
        public static final boolean softLimitEnabled = true;
        public static final Angle softLimitReverseMax = Units.Rotation.of(0);
        public static final Angle stowAngle = Units.Rotation.of(0);
        public static final Angle intakeAngle = Units.Rotation.of(0);
        public static final boolean isInverted = false;
        public static final Angle zeroOffset = Units.Rotation.of(0);
        public static final FeedbackSensor feedbackSensor = FeedbackSensor.kAbsoluteEncoder; 
        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;

        public enum IntakePivotModes {
            STOW(stowAngle),
            INTAKE(intakeAngle);

            public Angle angle;

            IntakePivotModes(Angle angle) {
                this.angle = angle;
            }

        }

    }

    public static class CANdleConstants {
        public static final int id = 0;

        // Team Colors
        public static final Color darkBlue = new Color(0, 0, 139);
        public static final Color cyanish = new Color(13, 228, 252);
        public static final Color purple = new Color(160, 0, 217);

        // Game Piece Colors
        public static final Color algaeGreen = new Color(69, 206, 162);
        public static final Color coralWhite = new Color(255, 230, 220);

        // Indicator Colors
        public static final Color green = new Color(56, 209, 0);
        public static final Color blue = new Color(8, 32, 255);
        public static final Color red = new Color(255, 0, 0);

        // Misc
        public static final Color black = new Color(0, 0, 0);
        public static final Color orange = new Color(255, 25, 0);

        // Animations
        public enum AnimationTypes {
            CORAL,
            ALGAE,
            BOTH,
            SLOW_WHITE,
            FLASHING_ORANGE,
            BLINK_RED,
            FLASHING_GREEN,
            OFF;
        }
    }
}
