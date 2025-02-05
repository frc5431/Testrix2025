package frc.robot.Subsytems.Manipulator;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointStates;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class ManipJoint extends REVMechanism {
	private ManipJointConfig config;
	private SparkMax motor;
	private DigitalInput beambreak;
	public Boolean attachted;

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
			configMaxMotion(ManipJointConstants.mm_velocity, ManipJointConstants.mm_maxAccel, ManipJointConstants.mm_error);
		}
	}

	public ManipJoint(SparkMax motor, DigitalInput beambreak, boolean attached) {
		super(motor, attached);
		ManipJointConfig config = new ManipJointConfig();
		this.motor = motor;
		this.mode = ManipJointPositions.STOW;
		// this.state = ManipJointStates;
		config.applySparkConfig(motor);

	}

	@Override
	public void periodic() {
		SmartDashboard.putString("ManipJoint Mode", this.getMode());
		SmartDashboard.putNumber("ManipJoint Output", this.getMotorOutput());
		SmartDashboard.putNumber("ManipJoint Current", this.getMotorCurrent());
		SmartDashboard.putNumber("ManipJoint Voltage", this.getMotorVoltage());
		SmartDashboard.putNumber("ManipJoint Position", this.getMotorPosition());
		SmartDashboard.putBoolean("ManipJoint Beambreak Status", this.getBeambreakStatus());
	}

	public void setManipJointState(ManipJointStates ManipJointState) {
		this.state = ManipJointState;
	}

	protected void runEnum(ManipJointPositions ManipJointmode) {
		this.mode = ManipJointmode;
		setMotorPosition(ManipJointmode.position);
	}

	// public Command runManipJointCommand(ManipJointPositions manipJointPositions)
	// {
	// return new StartEndCommand(() -> this.runEnum(manipJointPositions), () ->
	// this.runEnum(ManipJointPositions.STOW),
	// this)
	// .withName("ManipJoint.runEnum");
	// }

	@AutoLogOutput(key = "Manipulator/Joint/BeambreakS")
	public boolean getBeambreakStatus() {
		return beambreak.get();
	}

	@AutoLogOutput(key = "Manipulator/Joint/Position")
	public double getMotorPosition() {
		if (attached) {
			return motor.getEncoder().getPosition();
		}

		return 0;
	}

	@AutoLogOutput(key = "Manipulator/Joint/Voltage")
	public double getMotorVoltage() {
		if (attached) {
			return motor.getBusVoltage();
		}

		return 0;
	}

	@AutoLogOutput(key = "Manipulator/Joint/Current")
	public double getMotorCurrent() {
		if (attached) {
			return motor.getOutputCurrent();
		}

		return 0;
	}

	@AutoLogOutput(key = "Manipulator/Joint/Output")
	public double getMotorOutput() {
		if (attached) {
			return motor.getAppliedOutput();
		}

		return 0;
	}

	@AutoLogOutput(key = "Manipulator/Joint/Mode")
	public String getMode() {
		return this.mode.toString();
	}

	@AutoLogOutput(key = "Manipulator/Joint/State")
	public String getManipJointState() {
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
