package frc.robot.subsystems.mecanum;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface mecanumDriveIO {
    
    @AutoLog
    public static class mecanumDriveIOInputs {
        public double leftFrontPositionRad = 0.0;
        public double leftFrontVelocityRadPerSec = 0.0;
        public double leftFrontAppliedVolts = 0.0;
        public double leftFrontCurrentAmps = 0.0;

        public double leftBackPositionRad = 0.0;
        public double leftBackVelocityRadPerSec = 0.0;
        public double leftBackAppliedVolts = 0.0;
        public double leftBackCurrentAmps = 0.0;

        public double rightFrontPositionRad = 0.0;
        public double rightFrontVelocityRadPerSec = 0.0;
        public double rightFrontAppliedVolts = 0.0;
        public double rightFrontCurrentAmps = 0.0;

        public double rightBackPositionRad = 0.0;
        public double rightBackVelocityRadPerSec = 0.0;
        public double rightBackAppliedVolts = 0.0;
        public double rightBackCurrentAmps = 0.0;

        public Rotation2d gyroYaw = new Rotation2d();
        public double gyroRate = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(mecanumDriveIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double leftFrontVolts, double leftBackVolts, double rightFrontVolts, double rightBackVolts) {}

    /** Run closed loop at the specified velocity */
    public default void setVelocity(
        double leftFrontRadPerSec, double leftBackRadPerSec, double rightFrontRadPerSec, double rightBackRadPerSec, 
        double leftFrontFFVolts, double leftBackFFVolts, double rightFrontFFVolts, double rightBackFFVolts) {}
}
