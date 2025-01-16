package frc.robot.Subsytem.Intake;

import com.pathplanner.lib.config.RobotConfig;
import frc.robot.Util.Constants.IntakeConstants;

import frc.team5431.titan.core.subsystem.REVMechanism;

public class Intake extends REVMechanism {



    public class IntakeConfig extends Config {

        public IntakeConfig() {
            super("Intake", IntakeConstants.id, "rev");
        }
        

    }

    public Intake(boolean attached) {
        super(attached);
        if (attached) {
            
        }
    }

    @Override
    protected Config setConfig() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setConfig'");
    }
    
}
