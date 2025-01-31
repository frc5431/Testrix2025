package frc.robot.Util;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public final class Constants {

    public static class ControllerConstants {

        public enum ControlStates {
            ALGAE,
            CORAL,
        }

        public static final int driverPort = 0;
        public static final int operatorPort = 1;

        public static final double deadzone = 0.15;

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
        public static final int id = 0;
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

    public static class ManipulatorConstants {

        public enum ManipulatorStates {
            IDLE,
            LOCKED,
            FORWARD,
            REVERSE,
        }

        public static final boolean attached = true;
        public static final int id = 1;
        public static final double gearRatio = 0 / 0;
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

        public static final AngularVelocity intakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity idleSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);

        public static final MAXMotionPositionMode mm_positionMode = MAXMotionPositionMode.kMAXMotionTrapezoidal;
        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);

        public enum ManipulatorModes {
            IDLE(idleSpeed),
            FORWARD(intakeSpeed),
            FEED(feedSpeed),
            REVERSE(outtakeSpeed);

            public AngularVelocity speed;

            ManipulatorModes(AngularVelocity speed) {
                this.speed = speed;
            }

        }

    }

    public static class ManipJointConstants {

        public enum ManipJointStates {

        }

        public static final boolean attached = true;
        public static final int id = 1;
        public static final double gearRatio = 0 / 0;
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
}
