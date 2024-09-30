

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
/**
 * This drive implementation is for Talon SRXs driving brushed motors (e.g. CIMS) with no encoders
 * and no gyro.
 */
public class DriveIOTalonSRX implements DriveIO {
    private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);
    private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(DriveConstants.kLeftBackMotorPort);
    private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);
    private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(DriveConstants.kRightBackMotorPort);

    public DriveIOTalonSRX() {
        var config = new TalonSRXConfiguration();
        config.peakCurrentLimit = 60;
        config.peakCurrentDuration = 250;
        config.continuousCurrentLimit = 40;
        config.voltageCompSaturation = 10.0;
        leftLeader.configAllSettings(config);
        leftFollower.configAllSettings(config);
        rightLeader.configAllSettings(config);
        rightFollower.configAllSettings(config);

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
    inputs.leftAppliedVolts = leftLeader.getMotorOutputVoltage();
    inputs.leftCurrentAmps =
        new double[] {leftLeader.getSupplyCurrent(), leftFollower.getSupplyCurrent()};

    inputs.rightAppliedVolts = rightLeader.getMotorOutputVoltage();
    inputs.rightCurrentAmps =
        new double[] {rightLeader.getSupplyCurrent(), rightFollower.getSupplyCurrent()};
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.set(ControlMode.PercentOutput,leftVolts/12.0);
        rightLeader.set(ControlMode.PercentOutput,rightVolts/12.0);
    }

    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
        // Ignore for brushed motors, don't assume that the encoders are wired
    }
}
