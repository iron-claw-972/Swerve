package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO {

    private AHRS m_navX;

    public GyroIONavX() {
        m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = m_navX.isConnected();
        inputs.angle = m_navX.getAngle();
        inputs.velocity = m_navX.getRate();
    }

    public Rotation2d getRotation2d() {
        return m_navX.getRotation2d();
    }
    
}
