package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    
    public static class GyroIOInputs implements LoggableInputs {
        public boolean connected = false;
        public double angle = 0.0;
        public double velocity = 0.0;
    
        @Override
        public void toLog(LogTable table) {
            table.put("connected", connected);
            table.put("angle", angle);
            table.put("velocity", velocity);
        }
    
        @Override
        public void fromLog(LogTable table) {
            connected = table.getBoolean("connected", connected);
            angle = table.getDouble("angle", angle);
            velocity = table.getDouble("velocity", velocity);
        }
    }
    
    public void updateInputs(GyroIOInputs inputs);

    public default Rotation2d getRotation2d() { return null; }
    public default void addHeading(double heading) {}

}
