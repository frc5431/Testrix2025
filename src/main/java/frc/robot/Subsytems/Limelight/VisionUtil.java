package frc.robot.Subsytems.Limelight;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.team5431.titan.core.vision.Limelight;
import lombok.Setter;

public class VisionUtil {
    
    public static class VisionCommandConfig {
        
        public Limelight limelight;
        public int pipeline;
        @Setter public double p;
        @Setter public double maxOutput;
        @Setter public Distance distanceTolerance;
        @Setter public Angle angleTolerance;
        
        @Setter public Distance v_distanceSetpoint;
        @Setter public Distance h_distanceSetpoint;

        public void setLimeLight(Limelight limelight, int pipeline) {
            this.limelight = limelight;
            this.pipeline = pipeline;
        }
    }
    
}
