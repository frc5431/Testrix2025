package frc.robot.Subsytems.Elevator;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorStates;
import frc.team5431.titan.core.subsystem.CTREMechanism;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends CTREMechanism {

    public static class ElevatorConfig extends Config {

        public ElevatorConfig() {
            super("Elevator", ElevatorConstants.leftId, Constants.canbus);
            
            configVelocityPIDGains(0, ElevatorConstants.s, ElevatorConstants.p, ElevatorConstants.i, ElevatorConstants.d);
            configNeutralBrakeMode(ElevatorConstants.breakType);
            configGearRatio(ElevatorConstants.gearRatio);

            configMotionMagicPosition(ElevatorConstants.ff);
            configGravityType(ElevatorConstants.gravityType);

            configSupplyCurrentLimit(ElevatorConstants.supplyLimit, ElevatorConstants.useSupplyLimit);
            configStatorCurrentLimit(ElevatorConstants.stallLimit, ElevatorConstants.useStallLimit);
            configReverseTorqueCurrentLimit(ElevatorConstants.reverseTorqueLimit);
            configForwardTorqueCurrentLimit(ElevatorConstants.forwardTorqueLimit);

            configFeedbackSensorSource(ElevatorConstants.feedbackSensor, ElevatorConstants.rotationOffset.in(Rotation));
            configReverseSoftLimit(ElevatorConstants.maxReverseRotation.in(Rotation), ElevatorConstants.useRMaxRotation);
            configForwardSoftLimit(ElevatorConstants.maxFowardRotation.in(Rotation), ElevatorConstants.useFMaxRotation);
        }

    }

    private TalonFX leader;
    private TalonFX follower;

    private ElevatorConfig config = new ElevatorConfig();
    private CANcoder elevatorCANcoder;
    private CANrange canRange;
    private boolean attached;

    @Getter private ElevatorPositions position;
    @Getter @Setter private ElevatorStates states;

    /**
     * @param leader   json
     * @param follower zwach
     * @param attached is the mechanism in use
     */
    public Elevator(TalonFX leader, TalonFX follower, boolean attached) {
        super(leader, attached);
        this.leader = leader;
        this.follower = follower;
        this.attached = attached;
        elevatorCANcoder = new CANcoder(ElevatorConstants.canCoderId, Constants.canbus);
        canRange = new CANrange(ElevatorConstants.canRangeId, Constants.canbus);
        this.position = ElevatorPositions.STOW;
        this.states = ElevatorStates.STOWED;
        this.config.applyTalonConfig(leader);

        if (attached) {
            follower.setControl(new Follower(ElevatorConstants.leftId, ElevatorConstants.follwerInvert));
            Logger.recordOutput("Elevator/Mode", getPosition());
            Logger.recordOutput("Elevator/States", getStates());
            Logger.recordOutput("Elevator/Setpoint", getPosition().rotation.in(Rotation));
            Logger.recordOutput("Elevator/CANCoderPosition", elevatorCANcoder.getPosition().getValueAsDouble());
            Logger.recordOutput("Elevator/Position", leader.getPosition().getValueAsDouble());
            Logger.recordOutput("Elevator/Voltage", leader.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Elevator/Output", leader.getMotorOutputStatus().getValueAsDouble());
            Logger.recordOutput("Elevator/Acceleration", leader.getAcceleration().getValueAsDouble());
            Logger.recordOutput("Elevator/Velocity", leader.getVelocity().getValueAsDouble());
        } 

        if (ElevatorConstants.canRangeAttached) {
            Logger.recordOutput("Elevator/CanRange", canRange.getDistance().getValueAsDouble());
        }


    }

    public void periodic() {
        if (attached) {
            SmartDashboard.putString("Elevator Mode", getPosition().toString());
            SmartDashboard.putString("Elevator State", getStates().toString());
            SmartDashboard.putNumber("Elevator Setpoint", getPosition().rotation.in(Rotation));
            SmartDashboard.putNumber("Elevator Position", leader.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator/CANCoderPosition", elevatorCANcoder.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Position", follower.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Voltage", leader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Output", leader.getMotorOutputStatus().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Acceleration", leader.getAcceleration().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Velocity", leader.getVelocity().getValueAsDouble());
            if (attached && ElevatorConstants.canRangeAttached) {
                SmartDashboard.putNumber("CanRange Distance", canRange.getDistance().getValueAsDouble());
            }
        }
    }

    public void runEnum(ElevatorPositions position) {
        this.position = position;
        setMotorPosition(position.rotation);
    }

    public void runEnumMM(ElevatorPositions position) {
        this.position = position;
        setMMPosition(position.rotation);
        this.setDefaultCommand(null);
    }

    public void runEnumFOC(ElevatorPositions position) {
        this.position = position;
        setMMPositionFOC(position.rotation);
    }

    protected void stop() {
        if (attached) {
            leader.stopMotor();
        }
    }

    protected void setZero() {
        resetPosition();
    }

    protected void setZeroIntellegent() {

    }

    public Command runElevatorCommand(ElevatorPositions position) {
        return new RunCommand(() -> runEnum(position), this)
                .withName("Elevator.runEnum");
    }

    public Command runElevatorCommandMM(ElevatorPositions position) {
        return new RunCommand(() -> runEnumMM(position), this)
                .withName("Elevator.runEnumMM");
    }

    public Command runElevatorCommandFOC(ElevatorPositions position) {
        return new RunCommand(() -> runEnumFOC(position), this)
                .withName("Elevator.runEnumFOC");
    }

    public Command stopElevatorCommand(ElevatorPositions position) {
        return new RunCommand(() -> stop(), this)
                .withName("Elevator.STOP");
    }

    @Override
    protected Config setConfig() {
        if (attached) {
            setConfig(config);
        }
        return this.config;
    }

}
