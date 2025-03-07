package frc.robot.Subsytems.Elevator;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Util.Constants;
import frc.robot.Util.Constants.ElevatorConstants;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorStates;
import frc.team5431.titan.core.misc.Calc;
import frc.team5431.titan.core.subsystem.CTREMechanism;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends CTREMechanism {

    public static class ElevatorConfig extends Config {

        public ElevatorConfig() {
            super("Elevator", ElevatorConstants.leftId, Constants.canbus);

            configVelocityPIDGains(0, ElevatorConstants.s, ElevatorConstants.p, ElevatorConstants.i,
                    ElevatorConstants.d);
            configNeutralBrakeMode(ElevatorConstants.breakType);
            configGearRatio(ElevatorConstants.gearRatio);

            configMotionMagicPosition(ElevatorConstants.s);
            configGravityType(ElevatorConstants.gravityType);
            
            
            configSupplyCurrentLimit(ElevatorConstants.supplyLimit, ElevatorConstants.useSupplyLimit);
            configStatorCurrentLimit(ElevatorConstants.stallLimit, ElevatorConstants.useStallLimit);
            //configReverseTorqueCurrentLimit(ElevatorConstants.reverseTorqueLimit);
            //configForwardTorqueCurrentLimit(ElevatorConstants.forwardTorqueLimit);

            configFeedbackSensorSource(ElevatorConstants.feedbackSensor, ElevatorConstants.rotationOffset.in(Rotation));
            configReverseSoftLimit(ElevatorConstants.maxReverseRotation.in(Rotation),
                    ElevatorConstants.useRMaxRotation);
            configForwardSoftLimit(ElevatorConstants.maxFowardRotation.in(Rotation), ElevatorConstants.useFMaxRotation);
        }

    }

    private TalonFX leader;
    private TalonFX follower;

    private ElevatorConfig config = new ElevatorConfig();
    private CANcoder elevatorCANcoder;
    private CANrange canRange;
    private boolean attached;

    @Getter
    private ElevatorPositions position;
    @Getter
    @Setter
    private ElevatorStates states;

    /**
     * @param leader
     *            json
     * @param follower
     *            zwach
     * @param attached
     *            is the mechanism in use
     */
    public Elevator(TalonFX leader, TalonFX follower, boolean attached) {
        super(leader, attached);
           

        this.leader = leader;
        this.follower = follower;
        this.attached = attached;
        config.talonConfig.MotorOutput.withPeakForwardDutyCycle(ElevatorConstants.maxForwardOutput);
        config.talonConfig.MotorOutput.withPeakReverseDutyCycle(ElevatorConstants.maxReverseOutput  );

        leader.getConfigurator().apply(config.talonConfig);
        follower.getConfigurator().apply(config.talonConfig);
        follower.setControl(new Follower(ElevatorConstants.leftId , ElevatorConstants.follwerInvert));

        this.position = ElevatorPositions.STOW;
        this.states = ElevatorStates.STOWED;
        setZero();
        if (attached) {
            Logger.recordOutput("Elevator/Mode", getPosition());
            Logger.recordOutput("Elevator/States", getStates());
            Logger.recordOutput("Elevator/Setpoint", getPosition().rotation.in(Rotation));
            Logger.recordOutput("Elevator/Position", leader.getPosition().getValueAsDouble());
            Logger.recordOutput("Elevator/Voltage", leader.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Elevator/Output", leader.getMotorOutputStatus().getValueAsDouble());
            Logger.recordOutput("Elevator/Acceleration", leader.getAcceleration().getValueAsDouble());
            Logger.recordOutput("Elevator/Velocity", leader.getVelocity().getValueAsDouble());
        }


    }

    public void periodic() {
        if (attached) {
            SmartDashboard.putString("Elevator Mode", getPosition().toString());
            SmartDashboard.putString("Elevator State", getStates().toString());
            SmartDashboard.putNumber("Elevator Setpoint", getPosition().rotation.in(Rotation));
            SmartDashboard.putNumber("Leader Position", leader.getRotorPosition().getValueAsDouble());
            SmartDashboard.putNumber("Follower Position", follower.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Voltage", leader.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Output", leader.getClosedLoopOutput().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Acceleration", leader.getAcceleration().getValueAsDouble());
            SmartDashboard.putNumber("Elevator Velocity", leader.getVelocity().getValueAsDouble());

            if (ElevatorConstants.canRangeAttached) {
                SmartDashboard.putNumber("CanRange Distance", canRange.getDistance().getValueAsDouble());
            }

            if (ElevatorConstants.canCoderAttached) {
                SmartDashboard.putNumber("Elevator/CANCoderPosition",
                        elevatorCANcoder.getPosition().getValueAsDouble());
            }

        }
    }

    /**
     * Checks if the motor is reaching the rotational setpoint
     * 
     * @param target
     *            the target rotation angle
     * @param error
     *            allowed error in rotations (keep SMALL)
     * @return true if the motor's angle position is within the error of the target
     *         angle position
     */
    public boolean getPositionSetpointGoal(Angle target, Angle error) {
        if (attached) {
            if (Calc.approxEquals(leader.getRotorPosition().getValueAsDouble(), target.in(Rotation),
                    error.in(Rotation))) {
                return true;
            }
        }
        return false;
    }

    /**
     * @return if above or equal to the safe swing distance in rotations
     */
    public boolean isSwingSafe() {
        if (attached) {
            return leader.getRotorPosition().getValue().gte(ElevatorConstants.safeSwing);
        }
        return false;
    }

    public void stupid(){
        
    }

    public void runEnum(ElevatorPositions position) {
        this.position = position;
        setMMPosition(position.rotation);
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
            leader.stopMotor();
            follower.stopMotor();
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
        return new ElevatorConfig();
    }


}
