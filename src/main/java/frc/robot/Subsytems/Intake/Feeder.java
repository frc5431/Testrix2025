package frc.robot.Subsytems.Intake;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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


    public Feeder(SparkMax motor, boolean attachted) {
        super(motor, attachted);
        System.out.println("Feeder IS ALIVE");
        FeederConfig config = new FeederConfig();
        this.motor = motor;
        this.mode = FeederModes.IDLE;
        this.state = FeederStates.IDLE;
        config.applySparkConfig(motor);

        Logger.recordOutput("Feeder/Rollers/Mode", getMode());
        Logger.recordOutput("Feeder/Rollers/Setpoint", mode.speed.in(RPM));
        Logger.recordOutput("Feeder/Rollers/Velocity", getFeederState());
        Logger.recordOutput("Feeder/Rollers/Velocity", getMotorVelocity());
        Logger.recordOutput("Feeder/Rollers/Voltage", getMotorVoltage());
        Logger.recordOutput("Feeder/Rollers/Current", getMotorCurrent());
        Logger.recordOutput("Feeder/Rollers/Output", getMotorOutput());
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("getFeederState()", "getFeederState");
        SmartDashboard.putString("Feeder Mode", this.getMode());
        SmartDashboard.putNumber("Feeder Setpoint", mode.speed.in(RPM));
        SmartDashboard.putNumber("Feeder Output", getMotorOutput());
        SmartDashboard.putNumber("Feeder Current", getMotorCurrent());
        SmartDashboard.putNumber("Feeder Voltage", getMotorVoltage());
        SmartDashboard.putNumber("Feeder Velocity", getMotorVelocity());

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


    public void setFeederState(FeederStates FeederState) {
        this.state = FeederState;
    }

    protected void runEnum(FeederModes Feedermode) {
        this.mode = Feedermode;
        setVelocity(Feedermode.speed);
    }

    public Command runFeederCommand(FeederModes modes) {
        return new StartEndCommand(() -> this.runEnum(modes), () -> this.runEnum(FeederModes.IDLE), this)
                .withName("Feeder.runEnum");
    }


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
            setConfig(config);   }     
        return this.config;
    }

}
