package frc.robot.Subsytem.Intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;

import frc.robot.Util.Constants.IntakeConstants;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.IntakeConstants.IntakeStates;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class Intake extends REVMechanism {

    private IntakeConfig config;
    private SparkMax motor;
    public Boolean attachted;

    private IntakeModes intakeMode;
    private IntakeStates intakeStates;

    public static class IntakeConfig extends Config {

        public IntakeConfig() {
            super("Intake", IntakeConstants.id);
            configIdleMode(IntakeConstants.idleMode);
            configGearRatio(IntakeConstants.gearRatio);
            configMaxMotionPositionMode(IntakeConstants.mm_positionMode);
            configPIDGains(IntakeConstants.p, IntakeConstants.i, IntakeConstants.d);
            configSmartCurrentLimit(IntakeConstants.stallLimit, IntakeConstants.supplyLimit);
            configPeakOutput(IntakeConstants.maxForwardOutput, IntakeConstants.maxReverseOutput);
            configMaxMotion(IntakeConstants.mm_velocity, IntakeConstants.mm_maxAccel, IntakeConstants.mm_error);
        }
    }

    public Intake(SparkMax motor, IntakeConfig config, boolean attachted) {
        super(motor, attachted);
        this.config = config;
        this.motor = motor;
        setConfig(config);

    }

    @Override
    public void periodic() {

    }

    public void runEnum(IntakeModes intakemode) {
        this.intakeMode = intakemode;
        setVelocity(intakemode.speed);
    }

    @AutoLogOutput(key = "Intake/Rollers")
    public double getMotorVelocity() {
        if (attached) {
            return motor.getEncoder().getVelocity();
        }

        return 0;
    }

    /**
     * @return foward output, reverse output
     */
    @AutoLogOutput(key = "Intake/Rollers")
    public double getMotorOutput() {
        if (attached) {
            return motor.getAppliedOutput();
        }

        return 0;
    }

    @AutoLogOutput(key = "Intake/Rollers")
    public String getIntakeMode() {
        return this.intakeMode.toString();
    }

    @Override
    protected Config setConfig() {
        if (attachted) {
            config.applySparkConfig(motor);
        }
        return this.config;
    }

}