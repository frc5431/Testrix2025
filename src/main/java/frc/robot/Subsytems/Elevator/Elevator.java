package frc.robot.Subsytems.Elevator;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Util.Constants;
import frc.robot.Util.Constants.ElevatorConstants;
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
    
    public Elevator(TalonFX motor, boolean attached) {
        super(motor, attached);
        this.motor = motor;
        this.config.applyTalonConfig(motor);
    }

    public void periodic() {}


    @Override
    protected Config setConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setConfig'");
    }
    
}
