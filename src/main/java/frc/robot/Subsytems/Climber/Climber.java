package frc.robot.Subsytems.Climber;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Util.Constants.ClimberConstants;
import frc.robot.Util.Constants.ClimberConstants.ClimberModes;
import frc.robot.Util.Constants.ClimberConstants.ClimberStates;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class Climber extends REVMechanism {

    public boolean attached;
    public SysIdRoutine routine;

    private @Getter ClimberModes mode;
    private @Getter @Setter ClimberStates state;

    public static class ClimberConfig extends Config {

        public ClimberConfig() {
            super("Climber", ClimberConstants.id);
            configIdleMode(ClimberConstants.idleMode);
            configInverted(ClimberConstants.isInverted);
            configEncoderPosRatio(ClimberConstants.gearRatio);
            configMaxIAccum(ClimberConstants.maxIAccum);
            configMaxMotionPositionMode(ClimberConstants.mm_positionMode);
            configPIDGains(ClimberConstants.p, ClimberConstants.i, ClimberConstants.d);
            configSmartCurrentLimit(ClimberConstants.stallLimit, ClimberConstants.supplyLimit);
            configPeakOutput(ClimberConstants.maxForwardOutput, ClimberConstants.maxReverseOutput);
            configMaxMotion(ClimberConstants.mm_velocity, ClimberConstants.mm_maxAccel, ClimberConstants.mm_error);
        }
    }

    private ClimberConfig config = new ClimberConfig();

    public Climber(SparkMax motor, boolean attached) {
        super(motor, attached);
        this.motor = motor;
        this.attached = attached;
        this.mode = ClimberModes.STOW;
        this.state = ClimberStates.STOW;
        config.applySparkConfig(motor);

        Logger.recordOutput("Climber/Mode", getMode());
		Logger.recordOutput("Climber/State", getState());
		Logger.recordOutput("Climber/Setpoint", mode.angle.in(Rotations));
		Logger.recordOutput("Climber/Velocity", getMotorVelocity());
		Logger.recordOutput("Climber/Voltage", getMotorVoltage());
		Logger.recordOutput("Climber/Current", getMotorCurrent());
		Logger.recordOutput("Climber/Output", getMotorOutput());
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Climber Mode", getMode().toString());
        SmartDashboard.putString("Climber State", getState().toString());
		SmartDashboard.putNumber("Climber Setpoint", getMode().angle.in(Rotations));
		SmartDashboard.putNumber("Climber Velocity", getMotorVelocity());
		SmartDashboard.putNumber("Climber Voltage", getMotorVoltage());
		SmartDashboard.putNumber("Climber Current", getMotorCurrent());
		SmartDashboard.putNumber("Climber Output", getMotorOutput());

        switch (getMode()) {
            case STOW:
                setState(ClimberStates.STOW);
                break;
            case ALIGN:
                setState(ClimberStates.ALIGN);
                break;
            case CLIMB:
                setState(ClimberStates.CLIMB);
                break;
        }

    }

    protected void runRPM(AngularVelocity rpm) {
        this.mode = ClimberModes.CLIMB;
        setVelocity(rpm);
    }

    protected void runEnum(ClimberModes Climbermode) {
        this.mode = Climbermode;
        setMotorPosition(Climbermode.angle);
    }

    public Command runClimberCommand(AngularVelocity rpm) {
        return new StartEndCommand(() -> runRPM(rpm), () -> runRPM(RPM.of(0)), this);
    }

    public Command runClimberCommand(ClimberModes climberModes) {
        return run(() -> runEnum(climberModes)).withName("Climber.runEnum");
    }

    @Override
	protected Config setConfig() {
		if (attached) {
			setConfig(config);
		}
		return this.config;
	}

}