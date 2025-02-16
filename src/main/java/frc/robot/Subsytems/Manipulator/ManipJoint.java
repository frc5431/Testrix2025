package frc.robot.Subsytems.Manipulator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointStates;
import frc.robot.Util.Constants.ManipJointConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class ManipJoint extends REVMechanism {

	private ManipJointConfig config = new ManipJointConfig();
	private SparkMax motor;
	public boolean attached;

	@Getter private ManipJointPositions mode;
	@Getter @Setter private ManipJointStates state;

	public static class ManipJointConfig extends Config {

		public ManipJointConfig() {
			super("ManipJoint", ManipJointConstants.id);
			configIdleMode(ManipJointConstants.idleMode);
			configInverted(ManipJointConstants.isInverted);
			configEncoderPosRatio(ManipJointConstants.gearRatio);
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
		Logger.recordOutput("Manipulator/Joint/Setpoint", getMode().position.in(Rotation));
		Logger.recordOutput("Manipulator/Joint/State", getState());
		Logger.recordOutput("Manipulator/Joint/Velocity", getMotorVelocity());
		Logger.recordOutput("Manipulator/Joint/Voltage", getMotorVoltage());
		Logger.recordOutput("Manipulator/Joint/Current", getMotorCurrent());
		Logger.recordOutput("Manipulator/Joint/Output", getMotorOutput());
		Logger.recordOutput("Manipulator/Joint/Velocity", getMotorPosition());
	}

	@Override
	public void periodic() {
		SmartDashboard.putString("ManipJoint Mode", this.getMode().toString());
		SmartDashboard.putNumber("ManipJoint Setpoint", getMode().position.in(Rotations));
		SmartDashboard.putString("ManipJoint State", getState().toString());
		SmartDashboard.putNumber("ManipJoint Output", this.getMotorOutput());
		SmartDashboard.putNumber("ManipJoint Current", this.getMotorCurrent());
		SmartDashboard.putNumber("ManipJoint Voltage", this.getMotorVoltage());
		SmartDashboard.putNumber("ManipJoint Position", this.getMotorPosition());
	}

	public void setManipJointState(ManipJointStates ManipJointState) {
		this.state = ManipJointState;
	}

	protected void stop() {
		if (attached) {
			motor.stopMotor();
		}
	}

	protected void setZero() {
		resetPosition();
	}

	public void runEnum(ManipJointPositions ManipJointmode) {
		this.mode = ManipJointmode;
		setMotorPosition(ManipJointmode.position);
	}

	protected void runEnumMM(ManipJointPositions ManipJointmode) {
		this.mode = ManipJointmode;
		setMMPosition(ManipJointmode.position);
	}

	public Command runCleanerPivotCommand(ManipJointPositions ManipJointmode) {
		return new RunCommand(() -> this.runEnum(ManipJointmode), this)
				.withName("CleanPivot.runEnum");
	}

	public Command runCleanerPivotCommandMM(ManipJointPositions ManipJointmode) {
		return new RunCommand(() -> this.runEnumMM(ManipJointmode), this)
				.withName("CleanPivot.runEnumMM");
	}

	public Command stopCleanerPivotCommand() {
		return new RunCommand(() -> this.stop(), this)
				.withName("CleanPivot.STOP");
	}

	public Command cleanerPivotResetPositionCommand() {
		return new RunCommand(() -> this.setZero(), this)
				.withName("CleanPivot.setZero");
	}

	public double getMotorPosition() {
		if (attached) {
			return motor.getEncoder().getPosition();
		}

		return 0;
	}

	@Override
	protected Config setConfig() {
		if (attached) {
			config.applySparkConfig(motor);
		}
		return this.config;
	}
}
