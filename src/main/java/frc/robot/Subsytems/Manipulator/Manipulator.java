package frc.robot.Subsytems.Manipulator;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorStates;
import frc.robot.Util.Constants.ManipulatorConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class Manipulator extends REVMechanism {
  private ManipulatorConfig config;
  private SparkMax motor;
  public Boolean attachted;

  private ManipulatorModes mode;
  private ManipulatorStates state;

  public static class ManipulatorConfig extends Config {

    public ManipulatorConfig() {
      super("Manipulator", ManipulatorConstants.id);
      configIdleMode(ManipulatorConstants.idleMode);
      configInverted(ManipulatorConstants.isInverted);
      configGearRatio(ManipulatorConstants.gearRatio);
      configMaxIAccum(ManipulatorConstants.maxIAccum);
      configMaxMotionPositionMode(ManipulatorConstants.mm_positionMode);
      configPIDGains(ManipulatorConstants.p, ManipulatorConstants.i, ManipulatorConstants.d);
      configSmartCurrentLimit(ManipulatorConstants.stallLimit, ManipulatorConstants.supplyLimit);
      configPeakOutput(ManipulatorConstants.maxForwardOutput, ManipulatorConstants.maxReverseOutput);
      configMaxMotion(ManipulatorConstants.mm_velocity, ManipulatorConstants.mm_maxAccel,
          ManipulatorConstants.mm_error);
    }
  }

  public Manipulator(SparkMax motor, boolean attached, Boolean attachted) {
    super(motor, attached);
    ManipulatorConfig config = new ManipulatorConfig();
    this.motor = motor;
    this.mode = ManipulatorModes.IDLE;
    this.state = ManipulatorStates.IDLE;
    config.applySparkConfig(motor);

  }

  @Override
  protected Config setConfig() {
    if (attachted) {
      config.applySparkConfig(motor);
    }
    return this.config;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Mainpulator Mode", this.getMode());
    SmartDashboard.putNumber("Mainpulator Output", this.getMotorOutput());
    SmartDashboard.putNumber("Mainpulator Current", this.getMotorCurrent());
    SmartDashboard.putNumber("Mainpulator Voltage", this.getMotorVoltage());
    SmartDashboard.putNumber("Mainpulator Velocity", this.getMotorVelocity());

    switch (this.mode) {
      case IDLE:
        setManipulatorState(ManipulatorStates.IDLE);
        break;
      case INTAKE:
        setManipulatorState(ManipulatorStates.INTAKING);
        break;
      case OUTTAKE:
        setManipulatorState(ManipulatorStates.OUTTAKING);
        break;
      case FEED:
        setManipulatorState(ManipulatorStates.INTAKING);
        break;
    }
  }

  public void setManipulatorState(ManipulatorStates manipulatorStates) {
    this.state = manipulatorStates;
  }

  public void runEnum(ManipulatorModes manipulatorMode) {
    this.mode = manipulatorMode;
    setVelocity(manipulatorMode.speed);
  }

  @AutoLogOutput(key = "Manipulator/Rollers/Velocity")
  public double getMotorVelocity() {
    if (attached) {
      return motor.getEncoder().getVelocity();
    }

    return 0;
  }

  @AutoLogOutput(key = "Manipulator/Rollers/Output")
  public double getMotorOutput() {
    if (attached) {
      return motor.getAppliedOutput();
    }

    return 0;
  }

  @AutoLogOutput(key = "Manipulator/Rollers/Voltage")
  public double getMotorVoltage() {
    if (attached) {
      return motor.getBusVoltage();
    }

    return 0;
  }

  @AutoLogOutput(key = "Manipulator/Rollers/Current")
  public double getMotorCurrent() {
    if (attached) {
      return motor.getOutputCurrent();
    }

    return 0;
  }

  @AutoLogOutput(key = "Manipulator/Rollers/Mode")
  public String getMode() {
    return this.mode.toString();
  }

  @AutoLogOutput(key = "Manipulator/Rollers/State")
  public String getManipulatorState() {
    return this.state.toString();
  }

}
