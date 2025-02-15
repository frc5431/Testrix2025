package frc.robot.Subsytems.Intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util.Constants.FeederConstants;
import frc.robot.Util.Constants.FeederConstants.FeederModes;
import frc.robot.Util.Constants.FeederConstants.FeederStates;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class Feeder extends REVMechanism {

    private SparkMax motor;
    public boolean attachted;
    public SysIdRoutine routine;

    private FeederModes mode;
    private FeederStates state;

    public static class FeederConfig extends Config {

        public FeederConfig() {
            super("Feeder", FeederConstants.id);
            configIdleMode(FeederConstants.idleMode);
            configInverted(FeederConstants.isInverted);
            configGearRatio(FeederConstants.gearRatio);
            configMaxIAccum(FeederConstants.maxIAccum);
            configMaxMotionPositionMode(FeederConstants.mm_positionMode);
            configPIDGains(FeederConstants.p, FeederConstants.i, FeederConstants.d);
            configSmartCurrentLimit(FeederConstants.stallLimit, FeederConstants.supplyLimit);
            configPeakOutput(FeederConstants.maxForwardOutput, FeederConstants.maxReverseOutput);
            configMaxMotion(FeederConstants.mm_velocity, FeederConstants.mm_maxAccel, FeederConstants.mm_error);
        }
    }
    private FeederConfig config = new FeederConfig();

    public Feeder(SparkMax motor, FeederConfig config, boolean attachted) {
        super(motor, attachted);
        this.config = config;
        this.motor = motor;
        this.mode = FeederModes.IDLE;
        this.state = FeederStates.IDLE;
        config.applySparkConfig(motor);

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Feeder Mode", this.getMode());
        SmartDashboard.putString("getFeederState()", "getFeederState");
        switch (this.mode) {
            case IDLE:
                setFeederState(FeederStates.IDLE);
                break;
            case REVERSE:
                setFeederState(FeederStates.REVERSE);
                break;
            case FEED:
                setFeederState(FeederStates.FEEDING);
                break;
        }

    }


    public void setFeederState(FeederStates feederState) {
        this.state = feederState;
    }   

    protected void runEnum(FeederModes feedermode) {
        this.mode = feedermode;
        setVelocity(feedermode.speed);
    }

    public Command runFeederCommand(FeederModes feederModes) {
        return new StartEndCommand(() -> this.runEnum(feederModes), () -> this.runEnum(FeederModes.IDLE), this)
                .withName("Feeder.runEnum");
    }

    @AutoLogOutput(key = "Intake/Feeder")
    public double getMotorVelocity() {
        if (attached) {
            return motor.getEncoder().getVelocity();
        }

        return 0;
    }

    @AutoLogOutput(key = "Intake/Feeder")
    public double getMotorOutput() {
        if (attached) {
            return motor.getAppliedOutput();
        }

        return 0;
    }

    @AutoLogOutput(key = "Intake/Feeder")
    public String getMode() {
        return this.mode.toString();
    }

    @AutoLogOutput(key = "Intake/Feeder")
    public String getFeederState() {
        return this.state.toString();
    }

    @Override
    protected Config setConfig() {
        if (attachted) {
            setConfig(config);        }
        return this.config;
    }

}
