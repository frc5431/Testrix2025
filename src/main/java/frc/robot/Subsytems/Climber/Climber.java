package frc.robot.Subsytems.Climber;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util.Constants.ClimberConstants;
import frc.robot.Util.Constants.ClimberConstants.ClimberModes;
import frc.robot.Util.Constants.ClimberConstants.ClimberStates;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class Climber extends REVMechanism {

    private ClimberConfig config;
    private SparkMax motor;
    public boolean attachted;
    public SysIdRoutine routine;

    private ClimberModes mode;
    private ClimberStates state;

    public static class ClimberConfig extends Config {

        public ClimberConfig() {
            super("Climber", ClimberConstants.id);
            configIdleMode(ClimberConstants.idleMode);
            configInverted(ClimberConstants.isInverted);
            configGearRatio(ClimberConstants.gearRatio);
            configMaxIAccum(ClimberConstants.maxIAccum);
            configMaxMotionPositionMode(ClimberConstants.mm_positionMode);
            configPIDGains(ClimberConstants.p, ClimberConstants.i, ClimberConstants.d);
            configSmartCurrentLimit(ClimberConstants.stallLimit, ClimberConstants.supplyLimit);
            configPeakOutput(ClimberConstants.maxForwardOutput, ClimberConstants.maxReverseOutput);
            configMaxMotion(ClimberConstants.mm_velocity, ClimberConstants.mm_maxAccel, ClimberConstants.mm_error);
        }
    }

    public Climber(SparkMax motor, boolean attachted) {
        super(motor, attachted);
        ClimberConfig config = new ClimberConfig();
        this.motor = motor;
        this.mode = ClimberModes.STOW;
        this.state = ClimberStates.STOW;
        config.applySparkConfig(motor);

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Climber Mode", this.getMode());
        SmartDashboard.putString("getClimberState()", "getClimberState");
        switch (this.mode) {
            case STOW:
                setClimberState(ClimberStates.STOW);
                break;
            case ALIGN:
                setClimberState(ClimberStates.ALIGN);
                break;
            case CLIMB:
                setClimberState(ClimberStates.CLIMB);
                break;
        }

    }

    public void setClimberState(ClimberStates ClimberState) {
        this.state = ClimberState;
    }

    protected void runEnum(ClimberModes Climbermode) {
        this.mode = Climbermode;
        setMotorPosition(Climbermode.angle);
    }

    public Command runClimberCommand(ClimberModes climberModes) {
        // return new StartEndCommand(() -> this.runEnum(ClimberModes), () -> this.runEnum(ClimberModes), this).withName("Climber.runEnum");
        return run(() -> runEnum(climberModes)).withName("Climber.runEnum");
    }

    @AutoLogOutput(key = "Climber/Velocity")
    public double getMotorVelocity() {
        if (attached) {
            return motor.getEncoder().getVelocity();
        }

        return 0;
    }

    @AutoLogOutput(key = "Climber/Output")
    public double getMotorOutput() {
        if (attached) {
            return motor.getAppliedOutput();
        }

        return 0;
    }

    @AutoLogOutput(key = "Climber/Mode")
    public String getMode() {
        return this.mode.toString();
    }

    @AutoLogOutput(key = "Climber/State")
    public String getClimberState() {
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