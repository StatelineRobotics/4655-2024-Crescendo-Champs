package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public Rotation2d yaw = new Rotation2d();
        public double yawVelocity = 0.0; //degrees per second
    }

    public default void updateInputs(GyroIOInputs inputs){}
    
    public default void reset(){}
}
