package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Util.Constants.*;

public class Systems {

    private Intake intake;

    /* Kraken X60s */

    /* Neo 1.1s */
    private SparkMax intakeMotor;

    /* Neo 550s */

    public Systems() {

        /* Neo 1.1s */
        intakeMotor = new SparkMax(IntakeConstants.id, MotorType.kBrushless);
        
        intake = new Intake(intakeMotor, IntakeConstants.attached);

    }

    public Intake getIntake() {
        return intake;
    }

}
  