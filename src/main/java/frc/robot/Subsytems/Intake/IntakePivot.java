package frc.robot.Subsytems.Intake;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Util.Constants.IntakePivotConstants;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotModes;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotStates;
import frc.team5431.titan.core.subsystem.REVMechanism;
import lombok.Getter;
import lombok.Setter;

public class IntakePivot extends REVMechanism {

	public SparkMax motor;
	public SparkClosedLoopController controller;
	public AbsoluteEncoder absoluteEncoder;
	public double massKg;
	public boolean isShooter;

	@Getter @Setter public IntakePivotModes mode;
	@Getter @Setter public IntakePivotStates state;

	public static class PivotConfig extends Config {

		public PivotConfig() {
			super("IntakePivot", IntakePivotConstants.id);
			configSoftLimit(IntakePivotConstants.softLimitEnabled, IntakePivotConstants.softLimitForwardMax,
			IntakePivotConstants.softLimitReverseMax);
			configInverted(IntakePivotConstants.isInverted);
			configFeedbackSensorSource(IntakePivotConstants.feedbackSensor, IntakePivotConstants.zeroOffset);
			configPIDGains(IntakePivotConstants.p, IntakePivotConstants.i, IntakePivotConstants.d);
		}
	}
	private PivotConfig config = new PivotConfig();

	public IntakePivot(PivotConfig config, SparkMax motor, boolean attached) {
		super(motor, attached);

		this.config = config;
		this.motor = motor;
		this.mode = IntakePivotModes.STOW;
		this.state = IntakePivotStates.STOW;

		config.applySparkConfig(motor);

		Logger.recordOutput("Intake/Pivot/Mode", getMode());
		Logger.recordOutput("Intake/Pivot/State", getState());
		Logger.recordOutput("Intake/Pivot/SetPoint", getMode().angle.in(Rotation));
		Logger.recordOutput("Intake/Pivot/Output", getMotorOutput());
		Logger.recordOutput("Intake/Pivot/Position", absoluteEncoder.getPosition());
		Logger.recordOutput("Intake/Pivot/Current", getMotorCurrent());
		Logger.recordOutput("Intake/Pivot/Voltage", getMotorVoltage());
		Logger.recordOutput("Intake/Pivot/Velocity", getMotorVelocity());
	}

	public void periodic() {
		SmartDashboard.putString("Intake Pivot Mode", getMode().toString());
		SmartDashboard.putString("Intake Pivot State", getState().toString());
		SmartDashboard.putNumber("Intake Pivot SetPoint", getMode().angle.in(Rotation));
		SmartDashboard.putNumber("Intake Pivot Output", getMotorOutput());
		SmartDashboard.putNumber("Intake Pivot Position", absoluteEncoder.getPosition());
		SmartDashboard.putNumber("Intake Pivot Current", getMotorCurrent());
		SmartDashboard.putNumber("Intake Pivot Voltage", getMotorVoltage());
		SmartDashboard.putNumber("Intake Pivot Velocity", getMotorVelocity());
	}

	protected void runEnum(IntakePivotModes IntakePivotModes) {
		this.mode = IntakePivotModes;
		setMotorPosition(IntakePivotModes.angle);
	}

	protected void runEnumMM(IntakePivotModes IntakePivotModes) {
		this.mode = IntakePivotModes;
		setMMPosition(IntakePivotModes.angle);
	}

	protected void stop() {
		if (attached) {
			motor.stopMotor();
		}
	}

	protected void setZero() {
		resetPosition();
	}

	public Command runIntakePivotCommand(IntakePivotModes IntakePivotModes) {
		return new RunCommand(() -> this.runEnum(IntakePivotModes), this)
				.withName("IntakePivot.runEnum");
	}

	public Command runIntakePivotCommandMM(IntakePivotModes IntakePivotModes) {
		return new RunCommand(() -> this.runEnumMM(IntakePivotModes), this)
				.withName("IntakePivot.runEnumMM");
	}

	public Command stopIntakePivotCommand() {
		return new RunCommand(() -> this.stop(), this)
				.withName("IntakePivot.STOP");
	}

	public Command IntakePivotResetPositionCommand() {
		return new RunCommand(() -> this.setZero(), this)
				.withName("IntakePivot.setZero");
	}

	@Override
	protected Config setConfig() {
		if (attached) {
			setConfig(config);
		}
		return this.config;
	}

}
