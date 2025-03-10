package frc.robot.Util;

import frc.robot.Util.Constants.ElevatorConstants.ElevatorPositions;
import frc.robot.Util.Constants.IntakePivotConstants.IntakePivotModes;
import frc.robot.Util.Constants.ManipJointConstants.ManipJointPositions;
import lombok.*;
public class PresetPosition {

    private @Getter @Setter ManipJointPositions jointMode;
    private @Getter @Setter ElevatorPositions elevatorMode;
    private @Getter @Setter IntakePivotModes pivotMode;

    public PresetPosition(ElevatorPositions elevatorMode, ManipJointPositions jointMode, IntakePivotModes pivotMode) {
        this.elevatorMode = elevatorMode;
        this.jointMode = jointMode;
        this.pivotMode = pivotMode;
    }

    public PresetPosition(ElevatorPositions elevatorMode, ManipJointPositions jointMode) {
        this.elevatorMode = elevatorMode;
        this.jointMode = jointMode;
        this.pivotMode = IntakePivotModes.NONE;
    }
}
