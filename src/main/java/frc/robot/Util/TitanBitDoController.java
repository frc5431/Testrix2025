package frc.robot.Util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;

public class TitanBitDoController extends XboxController {
    /**
     * Trigger-Based 8BitDoController
     * 
     * @param port
     */
    public TitanBitDoController(final int port) {
        super(port);
    } 

    @Getter private Trigger LeftDPadLeft = new Trigger(() -> Math.round(getRawAxis(Axis.kLeftX.value)) == 0);
    @Getter private Trigger LeftDPadRight = new Trigger(() -> Math.round(getRawAxis(Axis.kLeftX.value)) == 1);
    @Getter private Trigger LeftDPadUp = new Trigger(() -> Math.round(getRawAxis(Axis.kLeftY.value)) == 0);
    @Getter private Trigger LeftDPadDown = new Trigger(() -> Math.round(getRawAxis(Axis.kLeftY.value)) == 1);

    @Getter private Trigger RightDPadRight = new Trigger(() -> Math.round(getRawAxis(Axis.kRightX.value)) == 1);
    @Getter private Trigger RightDPadLeft = new Trigger(() -> Math.round(getRawAxis(Axis.kRightX.value)) == 0);
    @Getter private Trigger RightDPadUp = new Trigger(() -> Math.round(getRawAxis(Axis.kRightY.value)) == 0);
    @Getter private Trigger RightDPadDown = new Trigger(() -> Math.round(getRawAxis(Axis.kRightY.value)) == 1);

    @Getter private Trigger L2Trigger = new Trigger(() -> Math.round(getRawAxis(Axis.kLeftTrigger.value)) == 1);
    @Getter private Trigger R2Trigger = new Trigger(() -> Math.round(getRawAxis(Axis.kRightTrigger.value)) == 1);

    @Getter private Trigger LBumper = new Trigger(() -> getLeftBumperButtonPressed());
    @Getter private Trigger RBumper = new Trigger(() -> getRightBumperButtonPressed());

    @Getter private Trigger povUp = new Trigger(() -> getPOV() == 0);
    @Getter private Trigger povLeft = new Trigger(() -> getPOV() == 90);
    @Getter private Trigger povDown = new Trigger(() -> getPOV() == 180);
    @Getter private Trigger povRight = new Trigger(() -> getPOV() == 270);

    @Getter private Trigger A = new Trigger(() -> getAButton());
    @Getter private Trigger B = new Trigger(() -> getBButton());
    @Getter private Trigger X = new Trigger(() -> getXButton());
    @Getter private Trigger Y = new Trigger(() -> getYButton());

    @Getter private Trigger Minus = new Trigger(() -> getBackButton());
    @Getter private Trigger Plus = new Trigger(() -> getStartButton());

}