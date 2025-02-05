package frc.robot.Subsytems.Manipulator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util.Constants.ManipulatorConstants;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorStates;
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
    
    Logger.recordOutput("Manipulator/Rollers/Velocity", getMotorVelocity());
    Logger.recordOutput("Manipulator/Rollers/Voltage", getMotorVoltage());
    Logger.recordOutput("Manipulator/Rollers/Current", getMotorCurrent());
    Logger.recordOutput("Manipulator/Rollers/Output", getMotorOutput());
    Logger.recordOutput("Manipulator/Rollers/Mode", getMode());
    Logger.recordOutput("Manipulator/Rollers/State", getManipulatorState());
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
    SmartDashboard.putString("Mainpulator Mode", getMode());
    SmartDashboard.putString("Manipulator State", getManipulatorState());
    SmartDashboard.putNumber("Mainpulator Output", getMotorOutput());
    SmartDashboard.putNumber("Mainpulator Current", getMotorCurrent());
    SmartDashboard.putNumber("Mainpulator Voltage", getMotorVoltage());
    SmartDashboard.putNumber("Mainpulator Velocity", getMotorVelocity());

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

  @AutoLogOutput(key = "Manipulator/Rollers/Mode")
  public String getMode() {
    return this.mode.toString();
  }

  @AutoLogOutput(key = "Manipulator/Rollers/State")
  public String getManipulatorState() {
    return this.state.toString();
  }

}
