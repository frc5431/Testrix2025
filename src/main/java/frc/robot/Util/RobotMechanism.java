package frc.robot.Util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;

public class RobotMechanism {
    
    @Getter public Mechanism2d elevator = new Mechanism2d(2, 8, new Color8Bit(Color.kYellow));
    @Getter public MechanismRoot2d elevatorBase = elevator.getRoot("Elevator", 2, 0);
    @Getter public MechanismLigament2d wrist = new MechanismLigament2d("Wrist", 3, 8, 2, new Color8Bit(Color.kPurple));
    
    public RobotMechanism() {
        elevatorBase.append(wrist);
        //wrist.append(elevatorBase);
    }












}
