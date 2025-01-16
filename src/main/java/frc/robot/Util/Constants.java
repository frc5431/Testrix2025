package frc.robot.Util;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.team5431.titan.core.subsystem.REVMechanism.SensorType;

public final class Constants {
 
    public static class ControllerConstant {
        public static final int d_port = 0;
        public static final int o_port = 1;

        public static final double deadzone = 0.15;
    }

    public static class IntakeConstants {
        
        public enum intakeModes {
            IDLE,
            INTAKE,
            FEED,
            OUTTAKE,
        }

        public static final boolean attached = true;
        public static final int id = 0;
        public static final double gearRatio = 0 / 0;
        public static final Current supplyLimit = Units.Amps.of(0);
        public static final Current stallLimit = Units.Amps.of(0);
        public static final IdleMode idleMode = IdleMode.kCoast;
        public static final Angle offset = Units.Rotation.of(0);
        public static final SensorType sensorType = SensorType.Relative;

        public static final double p = 0;
        public static final double i = 0;
        public static final double d = 0;

        public static final AngularVelocity intakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity outtakeSpeed = Units.RPM.of(0);
        public static final AngularVelocity feedSpeed = Units.RPM.of(0);
        public static final AngularVelocity error = Units.RPM.of(0);


        public static final AngularVelocity mm_maxAccel = Units.RPM.of(0);
        public static final AngularVelocity mm_velocity = Units.RPM.of(0);
        public static final AngularVelocity mm_error = Units.RPM.of(0);



    }



}
