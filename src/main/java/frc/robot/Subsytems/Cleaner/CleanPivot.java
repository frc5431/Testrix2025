package frc.robot.Subsytems.Cleaner;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.Constants.CleanPivotConstants;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotModes;
import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotStates;
import frc.robot.Util.Constants.CleanerConstants.CleanerModes;
import frc.robot.Util.Constants.CleanerConstants.CleanerStates; 
import frc.team5431.titan.core.subsystem.REVMechanism;

public class CleanPivot extends REVMechanism {

    public SparkMax motor;
    public SparkClosedLoopController controller;
    public AbsoluteEncoder absoluteEncoder;
    public CleanPivotModes mode;
    public CleanPivotStates state;
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

        this.config = config;
        this.motor = motor;
        this.mode = CleanPivotModes.STOW;
        this.state = CleanPivotStates.STOW;

        config.applySparkConfig(motor);
    }

    public void setManipJointState(CleanPivotStates cleanerPivotStates) {
		this.state = cleanerPivotStates;
	}

	protected void runEnum(CleanPivotModes cleanPivotModes) {
		this.mode = cleanPivotModes;
		setMotorPosition(cleanPivotModes.angle);
	}

    @AutoLogOutput(key = "Cleaner/Pivot/Position")
	public double getMotorPosition() {
		if (attached) {
			return motor.getEncoder().getPosition();
		}

		return 0;
	}

	@AutoLogOutput(key = "Cleaner/Pivot/Voltage")
	public double getMotorVoltage() {
		if (attached) {
			return motor.getBusVoltage();
		}

		return 0;
	}

	@AutoLogOutput(key = "Cleaner/Pivot/Current")
	public double getMotorCurrent() {
		if (attached) {
			return motor.getOutputCurrent();
		}

		return 0;
	}

	@AutoLogOutput(key = "Cleaner/Pivot/Output")
	public double getMotorOutput() {
		if (attached) {
			return motor.getAppliedOutput();
		}

		return 0;
	}

	@AutoLogOutput(key = "Cleaner/Pivot/Mode")
	public String getMode() {
		return this.mode.toString();
	}

	@AutoLogOutput(key = "Cleaner/Pivot/State")
	public String getManipJointState() {
		return this.state.toString();
	}

    @Override
    protected Config setConfig() {
        if (attached) {
            config.applySparkConfig(motor);
        }
        return this.config;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber(getName() + " encoder deg", absoluteEncoder.getPosition());
        SmartDashboard.putString(getName() + " Mode", this.mode.toString());
        SmartDashboard.putNumber(getName() + " output", motor.getAppliedOutput());

    }
}
