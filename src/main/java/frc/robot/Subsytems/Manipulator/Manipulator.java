package frc.robot.Subsytems.Manipulator;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;

import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorModes;
import frc.robot.Util.Constants.ManipulatorConstants.ManipulatorStates;
import frc.robot.Util.Constants.ManipulatorConstants;
import frc.team5431.titan.core.subsystem.REVMechanism;

public class Manipulator extends REVMechanism {
  private ManipulatorConfig config;
  private SparkMax motor;
  public Boolean attachted;

  private ManipulatorModes manipulatorMode;
  private ManipulatorStates manipulatorState;

  public static class ManipulatorConfig extends Config {

    public ManipulatorConfig() {
      super("Manipulator", ManipulatorConstants.id);
    }
  }

  public Manipulator(SparkMax motor, boolean attached, ManipulatorConfig config, Boolean attachted) {
    super(motor, attached);
    this.motor = motor;
    this.config = config;
    setConfig();
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

  }

  public void runEnum(ManipulatorModes manipulatorMode) {
    this.manipulatorMode = manipulatorMode;
    setVelocity(manipulatorMode.speed);
  }

  @AutoLogOutput(key = "Manipulator/Rollers")
  public double getMotorVelocity() {
    if (attached) {
      return motor.getEncoder().getVelocity();
    }

    return 0;
  }

  /**
   * @return foward output, reverse output
   */
  @AutoLogOutput(key = "Manipulator/Rollers")
  public double getMotorOutput() {
    if (attached) {
      return motor.getAppliedOutput();
    }

    return 0;
  }

  @AutoLogOutput(key = "Manipulator/Rollers")
  public String getIntakeMode() {
    return this.manipulatorMode.toString();
  }
}
