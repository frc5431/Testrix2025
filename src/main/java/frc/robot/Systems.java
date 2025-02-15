package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Subsytems.CANdle.TitanCANdle;
import frc.robot.Subsytems.Cleaner.CleanPivot;
import frc.robot.Subsytems.Cleaner.Cleaner;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Feeder;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.*;

public class Systems {

    private MotorType kBrushless = MotorType.kBrushless;

    private Intake intake;
    private Feeder feeder;
    private Elevator elevator;
    private Manipulator manipulator;
    private ManipJoint manipJoint;
    private Cleaner cleaner;
    private CleanPivot cleanPivot;
    private TitanCANdle candle;

    /* Kraken X60s */
    private TalonFX elevatorLeft;
    private TalonFX elevatorRight;

    /* Neo 1.1s */
    private SparkMax intakeMotor;
    private SparkMax cleanerMotor;
    private SparkMax cleanPivotMotor;
    private SparkMax manipJointMotor;
    private SparkMax feederMotor;

    /* Neo 550s */
    private SparkMax manipulatorMotor;

    public Systems() {

        /* Kraken X60s */
        elevatorLeft = new TalonFX(ElevatorConstants.leftId, Constants.canbus);
        elevatorRight = new TalonFX(ElevatorConstants.rightId, Constants.canbus);

        /* Neo 1.1s */
        intakeMotor = new SparkMax(IntakeConstants.id, kBrushless);
        cleanerMotor = new SparkMax(CleanerConstants.id, kBrushless);
        cleanPivotMotor = new SparkMax(CleanPivotConstants.id, kBrushless);
        manipJointMotor = new SparkMax(ManipJointConstants.id, kBrushless);
        feederMotor = new SparkMax(FeederConstants.id, kBrushless);

        /* Neo 550s */
        manipulatorMotor = new SparkMax(ManipulatorConstants.id, kBrushless);

        /*----------*/
        feeder = new Feeder(feederMotor, FeederConstants.attached);
        intake = new Intake(intakeMotor, IntakeConstants.attached);
        elevator = new Elevator(elevatorLeft, elevatorRight, ElevatorConstants.attached);
        cleaner = new Cleaner(cleanerMotor, CleanerConstants.attached);
        cleanPivot = new CleanPivot(cleanPivotMotor, CleanPivotConstants.attached);
        manipulator = new Manipulator(manipulatorMotor, ManipulatorConstants.attached);
        manipJoint = new ManipJoint(manipJointMotor, ManipJointConstants.attached);
        candle = new TitanCANdle();
    }

    public Intake getIntake() {
        return intake;
    }

    public Elevator getElevator() {
        return elevator;
    }

    public Manipulator getManipulator() {
        return manipulator;
    }

    public ManipJoint getManipJoint() {
        return manipJoint;
    }

    public Cleaner getCleaner() {
        return cleaner;
    }
    
    public CleanPivot getCleanPivot() {
        return cleanPivot;
    }

    public Feeder getFeeder() {
        return feeder;
    }

    public TitanCANdle getCandle() {
        return candle;
    }

}
