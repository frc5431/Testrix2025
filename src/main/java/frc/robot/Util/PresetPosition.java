package frc.robot.Util;

import frc.robot.Util.Constants.CleanPivotConstants.CleanPivotModes;
import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import lombok.Setter;
import lombok.Getter;
public class PresetPosition {

    @Getter @Setter private CleanPivotModes pivotMode;
    @Getter @Setter private ManipJointPositions jointMode;
    @Getter @Setter private ElevatorPositions elevatorMode;

    public PresetPosition(ElevatorPositions elevatorMode, ManipJointPositions jointMode, CleanPivotModes pivotMode) {
        this.elevatorMode = elevatorMode;
        this.jointMode = jointMode;
        this.pivotMode = pivotMode;
    }

    public PresetPosition(ElevatorPositions elevatorMode, CleanPivotModes pivotMode, ManipJointPositions jointMode) {
        this.elevatorMode = elevatorMode;
        this.jointMode = jointMode;
        this.pivotMode = pivotMode;
    }

    public PresetPosition(ElevatorPositions elevatorMode, ManipJointPositions jointMode) {
        this.elevatorMode = elevatorMode;
        this.jointMode = jointMode;
        this.pivotMode = CleanPivotModes.STOW;
    }

    public PresetPosition(ElevatorPositions elevatorMode, CleanPivotModes pivotMode) {
        this.elevatorMode = elevatorMode;
        this.jointMode = ManipJointPositions.STOW;
        this.pivotMode = pivotMode;
    }
}
