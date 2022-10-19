package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;

public class GyroIOPigeon implements GyroIO {

    private WPI_Pigeon2 m_pigeon;

    public GyroIOPigeon() {
      m_pigeon = new WPI_Pigeon2(Constants.drive.kPigeon, Constants.kRioCAN);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // inputs.connected = ; // TODO: figure out how to see pigeon status. Pigeon 1 has getState() but not pigeon2?
        inputs.angle = m_pigeon.getAngle();
        inputs.velocity = m_pigeon.getRate();
    }

    public Rotation2d getRotation2d() {
        return m_pigeon.getRotation2d();
    }
    
}
