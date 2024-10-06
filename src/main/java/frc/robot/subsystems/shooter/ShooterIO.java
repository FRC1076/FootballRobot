package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double leftPositionRads = 0.0;
        public double leftVelocityRadsPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        
        public double rightPositionRads = 0.0;
        public double rightVelocityRadsPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ShooterIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double leftVolts, double rightVolts) {}

    /** Run closed loop at the specified velocity. */
    public default void setVelocity(
        double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {}
}
