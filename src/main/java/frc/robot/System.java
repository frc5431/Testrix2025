package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Subsytems.Cleaner.Cleaner;
import frc.robot.Subsytems.Cleaner.Cleaner.CleanerConfig;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Intake.Intake.IntakeConfig;
import frc.robot.Util.Constants.*;

public class System {

    private Intake intake;

    private Cleaner cleaner;

    private IntakeConfig intakeConfig = new IntakeConfig();

    private CleanerConfig cleanerConfig = new CleanerConfig();
    /*
     * Kraken X60s
     */

    /*
     * Neo 1.1s
     */
    private SparkMax intakeMotor;

    private SparkMax cleanerMotor;
    /*
     * Neo 550s
     */

    public System() {

        /*
         * Neo 1.1s
         */
        intakeMotor = new SparkMax(IntakeConstants.id, MotorType.kBrushless);
        
        cleanerMotor = new SparkMax(CleanerConstants.id, MotorType.kBrushless);
        
        intake = new Intake(intakeMotor, intakeConfig, IntakeConstants.attached);

        cleaner = new Cleaner(cleanerMotor, cleanerConfig, CleanerConstants.attached);

    }

    public Intake getIntake() {
        return intake;
    }

    public Cleaner getCleaner() {
        return cleaner;
    }

}
