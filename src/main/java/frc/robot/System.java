package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Subsytems.Climber.Climber;
import frc.robot.Subsytems.Climber.Climber.ClimberConfig;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Intake.Intake.IntakeConfig;
import frc.robot.Util.Constants.*;

public class System {

    private Intake intake;

    private IntakeConfig intakeConfig = new IntakeConfig();

    private Climber climber;
    private ClimberConfig climberConfig;

    /*
     * Kraken X60s
     */

    /*
     * Neo 1.1s
     */
    private SparkMax intakeMotor;
    private SparkMax climberMotor;

    /*
     * Neo 550s
     */

    public System() {

        /*
         * Neo 1.1s
         */
        intakeMotor = new SparkMax(IntakeConstants.id, MotorType.kBrushless);
        climberMotor = new SparkMax(ClimberConstants.id, MotorType.kBrushless);
        
        intake = new Intake(intakeMotor, intakeConfig, IntakeConstants.attached);
        climber = new Climber(climberMotor, ClimberConstants.attached);

    }

    public Intake getIntake() {
        return intake;
    }

    public Climber getClimber(){
        return climber;
    }

}
