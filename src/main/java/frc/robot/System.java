package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Subsytem.Intake.Intake;
import frc.robot.Subsytem.Intake.Intake.IntakeConfig;
import frc.robot.Util.Constants.*;

public class System {

    private Intake intake;


    private IntakeConfig intakeConfig = new IntakeConfig();

    private SparkMax intakeMotor;

    public System() {

        intakeMotor = new SparkMax(IntakeConstants.id, MotorType.kBrushless);

        intake = new Intake(intakeMotor, intakeConfig, IntakeConstants.attached);

    }

    public Intake getIntake() {
        return intake;
    }

}
