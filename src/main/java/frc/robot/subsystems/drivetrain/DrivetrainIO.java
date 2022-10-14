package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface DrivetrainIO {
    
    public static class DrivetrainIOInputs implements LoggableInputs {
        public double[] robotPose = {0.0, 0.0, 0.0};
    
        @Override
        public void toLog(LogTable table) {
            table.put("robotPose", robotPose);
        }
    
        @Override
        public void fromLog(LogTable table) {
            robotPose = table.getDoubleArray("robotPose", robotPose);
        }
    }
    
    public void updateInputs(DrivetrainIOInputs inputs);

    public default double[] getRobotPose() { return null; }

}
