package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util.Constants.IntakeConstants;
import frc.robot.Util.Constants.IntakeConstants.IntakeModes;
import frc.robot.Util.Constants.IntakeConstants.IntakeStates;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class Intake extends REVMechanism {

    private IntakeConfig config;
    private SparkMax motor;
    public boolean attachted;
    public SysIdRoutine routine;

    private IntakeModes mode;
    private IntakeStates state;

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
        this.mode = IntakeModes.IDLE;
        this.state = IntakeStates.IDLE;
        setConfig();   
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake Mode", this.getMode());
        switch (this.mode) {
            case IDLE:
                setIntakeState(IntakeStates.IDLE);
                break;
            case INTAKE:
                setIntakeState(IntakeStates.INTAKING);
                break;
            case OUTTAKE:
                setIntakeState(IntakeStates.OUTTAKING);
                break;
            case FEED:
                setIntakeState(IntakeStates.FEEDING);
                break;
        }
        
    }

    public void setIntakeState(IntakeStates intakeState) {
        this.state = intakeState;
    }

    protected void runEnum(IntakeModes intakemode) {
        this.mode = intakemode;
        setVelocity(intakemode.speed);
    }

    public Command runIntakeCommand(IntakeModes intakeModes) {
        return new StartEndCommand(() -> this.runEnum(intakeModes), () -> this.runEnum(IntakeModes.IDLE), this).withName("Intake.runEnum");
    }

    @AutoLogOutput(key = "Intake/Rollers")
    public double getMotorVelocity() {
        if (attached) {
            return motor.getEncoder().getVelocity();
        }

        return 0;
    }

    @AutoLogOutput(key = "Intake/Rollers")
    public double getMotorOutput() {
        if (attached) {
            return motor.getAppliedOutput();
        }

        return 0;
    }

    @AutoLogOutput(key = "Intake/Rollers")
    public String getMode() {
        return this.mode.toString();
    }

    @AutoLogOutput(key = "Intake/Rollers")
    public String getIntakeState() {
        return this.state.toString();
    }

    @Override
    protected Config setConfig() {
        if (attachted) {
            config.applySparkConfig(motor);
        }
        return this.config;
    }

}