package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Subsytems.CANdle.TitanCANdle;
import frc.robot.Subsytems.Cleaner.CleanPivot;
import frc.robot.Subsytems.Cleaner.Cleaner;
import frc.robot.Subsytems.Climber.Climber;
import frc.robot.Subsytems.Drivebase.Drivebase;
import frc.robot.Subsytems.Elevator.Elevator;
import frc.robot.Subsytems.Intake.Feeder;
import frc.robot.Subsytems.Intake.Intake;
import frc.robot.Subsytems.Limelight.Vision;
import frc.robot.Subsytems.Intake.IntakePivot;
import frc.robot.Subsytems.Manipulator.ManipJoint;
import frc.robot.Subsytems.Manipulator.Manipulator;
import frc.robot.Util.Constants;
import frc.robot.Util.TunerConstants;
import frc.robot.Util.Constants.*;
import lombok.Getter;

public class Systems {

    private Intake intake;
    private Elevator elevator;
    private Manipulator manipulator;
    private ManipJoint manipJoint;
    private Cleaner cleaner;
    private CleanPivot cleanPivot;
    private TitanCANdle candle;
    private Vision vision;

    /* Kraken X60s */
    private TalonFX elevatorLeft;
    private TalonFX elevatorRight;

    /* Neo 1.1s */
    private SparkMax intakeMotor;
    private SparkMax cleanerMotor;
    private SparkMax cleanPivotMotor;
    private SparkMax manipJointMotor;
    private SparkMax feederMotor;
    private SparkMax intakePivotMotor;
    private SparkMax climberMotor;

    /* Neo 550s */
    private SparkMax manipulatorMotor;

    public Systems() {

        /* Kraken X60s */
        elevatorLeft = new TalonFX(ElevatorConstants.leftId, Constants.canbus);
        elevatorRight = new TalonFX(ElevatorConstants.rightId, Constants.canbus);

        /* Neo 1.1s */
        intakeMotor = new SparkMax(IntakeConstants.id, brushless);
        intakePivotMotor = new SparkMax(IntakePivotConstants.id, brushless);
        cleanerMotor = new SparkMax(CleanerConstants.id, brushless);
        cleanPivotMotor = new SparkMax(CleanPivotConstants.id, brushless);
        manipJointMotor = new SparkMax(ManipJointConstants.id, brushless);
        feederMotor = new SparkMax(FeederConstants.id, brushless);
        climberMotor = new SparkMax(ClimberConstants.id, brushless);

        /* Neo 550s */
        manipulatorMotor = new SparkMax(ManipulatorConstants.id, brushless);

        /*----------*/
        feeder = new Feeder(feederMotor, FeederConstants.attached);
        intake = new Intake(intakeMotor, IntakeConstants.attached);
        intakePivot = new IntakePivot(intakePivotMotor, IntakePivotConstants.attached);
        elevator = new Elevator(elevatorLeft, elevatorRight, ElevatorConstants.attached);
        cleaner = new Cleaner(cleanerMotor, CleanerConstants.attached);
        cleanPivot = new CleanPivot(cleanPivotMotor, CleanPivotConstants.attached);
        manipulator = new Manipulator(manipulatorMotor, ManipulatorConstants.attached);
        manipJoint = new ManipJoint(manipJointMotor, ManipJointConstants.attached);
        climber = new Climber(climberMotor, ClimberConstants.attached);
        candle = new TitanCANdle();
        
        drivebase = new Drivebase(
                TunerConstants.DrivetrainConstants, 
                TunerConstants.FrontLeft, TunerConstants.FrontRight,
                TunerConstants.BackLeft, TunerConstants.BackRight);
    }
}
