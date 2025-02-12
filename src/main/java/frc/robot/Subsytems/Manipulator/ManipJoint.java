package frc.robot.Subsytems.Manipulator;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointStates;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class ManipJoint extends REVMechanism {
	
	private ManipJointConfig config = new ManipJointConfig();
	private SparkMax motor;
	public boolean attached;

	private ManipJointPositions mode;
	private ManipJointStates state;

	public static class ManipJointConfig extends Config {

		public ManipJointConfig() {
			super("ManipJoint", ManipJointConstants.id);
			configIdleMode(ManipJointConstants.idleMode);
			configInverted(ManipJointConstants.isInverted);
			configGearRatio(ManipJointConstants.gearRatio);
			configMaxIAccum(ManipJointConstants.maxIAccum);
			configMaxMotionPositionMode(ManipJointConstants.mm_positionMode);
			configPIDGains(ManipJointConstants.p, ManipJointConstants.i, ManipJointConstants.d);
			configSmartCurrentLimit(ManipJointConstants.stallLimit, ManipJointConstants.supplyLimit);
			configPeakOutput(ManipJointConstants.maxForwardOutput, ManipJointConstants.maxReverseOutput);
			configMaxMotion(ManipJointConstants.mm_velocity, ManipJointConstants.mm_maxAccel,
					ManipJointConstants.mm_error);
		}
	}

	public ManipJoint(SparkMax motor, boolean attached) {
		super(motor, attached);
		this.motor = motor;
		this.attached = attached;
		this.mode = ManipJointPositions.STOW;
		this.state = ManipJointStates.STOWED;
		config.applySparkConfig(motor);

		Logger.recordOutput("Manipulator/Joint/Mode", getMode());
		Logger.recordOutput("Manipulator/Joint/Setpoint", this.mode.position.in(Rotation));
		Logger.recordOutput("Manipulator/Joint/State", getManipJointState());
		Logger.recordOutput("Manipulator/Joint/Velocity", getMotorVelocity());
		Logger.recordOutput("Manipulator/Joint/Voltage", getMotorVoltage());
		Logger.recordOutput("Manipulator/Joint/Current", getMotorCurrent());
		Logger.recordOutput("Manipulator/Joint/Output", getMotorOutput());
		Logger.recordOutput("Manipulator/Joint/Velocity", getMotorPosition());
	}

	@Override
	public void periodic() {
		SmartDashboard.putString("ManipJoint Mode", this.getMode());
		SmartDashboard.putNumber("ManipJoint Setpoint", this.mode.position.in(Rotation));
		SmartDashboard.putString("ManipJoint State", getManipJointState());
		SmartDashboard.putNumber("ManipJoint Output", this.getMotorOutput());
		SmartDashboard.putNumber("ManipJoint Current", this.getMotorCurrent());
		SmartDashboard.putNumber("ManipJoint Voltage", this.getMotorVoltage());
		SmartDashboard.putNumber("ManipJoint Position", this.getMotorPosition());
	}

	public void setManipJointState(ManipJointStates ManipJointState) {
		this.state = ManipJointState;
	}

	public void runEnum(ManipJointPositions ManipJointmode) {
		this.mode = ManipJointmode;
		setMotorPosition(ManipJointmode.position);
	}

	protected void runEnumMM(ManipJointPositions ManipJointmode) {
		this.mode = ManipJointmode;
		setMMPosition(ManipJointmode.position);
	}
	
	public double getMotorPosition() {
		if (attached) {
			return motor.getEncoder().getPosition();
		}

		return 0;
	}

	public String getMode() {
		return this.mode.toString();
	}

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
}
