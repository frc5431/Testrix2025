package frc.robot.Subsytems.Elevator;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Util.Constants;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorStates;
import frc.team5431.titan.core.subsystem.CTREMechanism;

public class Elevator extends CTREMechanism {

    public static class ElevatorConfig extends Config {

        public ElevatorConfig() {
            super("Elevator", ElevatorConstants.id, Constants.canbus);
            configGravityType(ElevatorConstants.gravityType);
            configFeedForwardGains(ElevatorConstants.s, ElevatorConstants.v, ElevatorConstants.a, ElevatorConstants.g);
            configReverseSoftLimit(ElevatorConstants.maxReverseRotation.in(Rotation), ElevatorConstants.useRMaxRotation);
            configForwardTorqueCurrentLimit(ElevatorConstants.forwardCurrentLimit);
            configReverseTorqueCurrentLimit(ElevatorConstants.reverseCurrentLimit);
            configMotionMagicPosition(ElevatorConstants.ff);
            configFeedbackSensorSource(ElevatorConstants.feedbackSensor, ElevatorConstants.rotationOffset.in(Rotation));
            configGearRatio(ElevatorConstants.gearRatio);
            configNeutralBrakeMode(ElevatorConstants.breakType);
            configPIDGains(ElevatorConstants.p, ElevatorConstants.i, ElevatorConstants.d);
            configForwardSoftLimit(ElevatorConstants.maxFowardRotation.in(Rotation), ElevatorConstants.useFMaxRotation);
        }

    }


    private TalonFX motor;
    private ElevatorConfig config;
    private boolean attached;

    private ElevatorPositions position;
    private ElevatorStates states;

    
    public Elevator(TalonFX motor, boolean attached) {
        super(motor, attached);
        this.motor = motor;
        this.attached = attached;
        ElevatorConfig config = new ElevatorConfig();
        this.config = config;
        this.config.applyTalonConfig(motor);
    }

    public void periodic() {

    }

    public void runEnum(ElevatorPositions position) {
        this.position = position;
        setMotorPosition(position.rotation);
    }

    public void runEnumMM(ElevatorPositions position) {
        this.position = position;
        setMMPosition(position.rotation);
    }

    public void runEnumFOC(ElevatorPositions position) {
        this.position = position;
        setMMPositionFOC(position.rotation);
    }

    protected void stop() {
        if (attached) {
            motor.stopMotor();
        }
    }

    protected void setZero() {
        if (attached) {
            resetPosition();
        }
    }

    


    @Override
    protected Config setConfig() {
        if (attached) {
            config.applyTalonConfig(motor); 
        }
        return this.config;
    }
    
}
