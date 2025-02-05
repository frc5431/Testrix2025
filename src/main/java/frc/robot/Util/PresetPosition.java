package frc.robot.Util;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;

public class PresetPosition {

    private Angle cleanerAngle;
    private Angle manipulatorAngle;
    private Angle elevatorRot;
    
    public Optional<Boolean> useCleaner;

    public PresetPosition(Angle manipulatorAngle, Angle elevatorRot) {
        this.manipulatorAngle = manipulatorAngle;
        this.elevatorRot = elevatorRot;
    }

    public PresetPosition(Angle cleanerAngle, Optional<Boolean> useCleaner) {
        this.cleanerAngle = cleanerAngle;
        this.useCleaner = useCleaner;
    }

    public PresetPosition(Angle cleanerAngle, Angle manipulatorAngle, Angle elevatorRot) {
        this.cleanerAngle = cleanerAngle;
        this.manipulatorAngle = manipulatorAngle;
        this.elevatorRot = elevatorRot;
    }

    public PresetPosition(Angle manipulatorAngle, Angle elevatorRot, Optional<Boolean> useCleaner) {
        this.manipulatorAngle = manipulatorAngle;
        this.elevatorRot = elevatorRot;
        this.useCleaner = useCleaner;
    }

    public PresetPosition(Angle cleanerAngle, Angle manipulatorAngle, Angle elevatorRot, Optional<Boolean> useCleaner) {
        this.cleanerAngle = cleanerAngle;
        this.manipulatorAngle = manipulatorAngle;
        this.elevatorRot = elevatorRot;
        this.useCleaner = useCleaner;
    }

    public Angle getElevAngle() {
        return elevatorRot;
    }

    public Angle getCleanerAngle() {
        return cleanerAngle;
    }

    public Angle getManipAngle() {
        return manipulatorAngle;
    }

    public Optional<Boolean> usesCleaner() {
        return useCleaner;
    }

}
