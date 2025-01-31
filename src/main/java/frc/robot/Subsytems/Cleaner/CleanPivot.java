package frc.robot.Subsytems.Cleaner;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Util.Constants.CleanPivotConstants;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotModes;
import frc.robot.Util.Constants.CleanerConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class CleanPivot extends REVMechanism {

    public SparkMax motor;
    public SparkClosedLoopController controller;
    public AbsoluteEncoder absoluteEncoder;
    public CleanPivotModes mode;
    public double massKg;
    public boolean isShooter;

    public static class PivotConfig extends Config {

        public PivotConfig() {
            super("CleanPivot", CleanPivotConstants.id);
            configSoftLimit(CleanPivotConstants.softLimitEnabled, CleanPivotConstants.softLimitForwardMax, CleanPivotConstants.softLimitReverseMax);
            configInverted(CleanPivotConstants.isInverted);
            configFeedbackSensorSource(CleanPivotConstants.feedbackSensor, CleanPivotConstants.zeroOffset);
            configPIDGains(CleanPivotConstants.p, CleanPivotConstants.i, CleanPivotConstants.d);
        }
    }
    
    public CleanPivot(PivotConfig config, SparkMax motor, boolean attached){
        super(motor, attached);
        this.motor = motor;

        config.applySparkConfig(motor);
    }

    @Override
    protected Config setConfig() {
        if (attached) {
            config.applySparkConfig(motor);
        }
        return this.config;
    }
}
