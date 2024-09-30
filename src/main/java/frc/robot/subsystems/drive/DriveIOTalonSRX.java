

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants.DriveConstants;
/**
 * This drive implementation is for Talon SRXs driving brushed motors (e.g. CIMS) with no encoders.
 */
public class DriveIOTalonSRX implements DriveIO {
    private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);
    private final WPI_TalonSRX leftFollower = new WPI_TalonSRX(DriveConstants.kLeftBackMotorPort);
    private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);
    private final WPI_TalonSRX rightFollower = new WPI_TalonSRX(DriveConstants.kRightBackMotorPort);
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    public DriveIOTalonSRX() {
        var config = new TalonSRXConfiguration();
        config.peakCurrentLimit = DriveConstants.Electrical.kPeakCurrentLimit;
        config.peakCurrentDuration = DriveConstants.Electrical.kPeakCurrentDuration;
        config.continuousCurrentLimit = DriveConstants.Electrical.kContinuousCurrentLimit;
        config.voltageCompSaturation = DriveConstants.Electrical.kVoltageComp; //"Full output" will scale to 12 volts, regardless of actual battery voltage level
        
        leftLeader.configAllSettings(config);
        leftFollower.configAllSettings(config);
        rightLeader.configAllSettings(config);
        rightFollower.configAllSettings(config);
        leftLeader.enableVoltageCompensation(DriveConstants.Electrical.kVoltageCompEnabled);
        leftFollower.enableVoltageCompensation(DriveConstants.Electrical.kVoltageCompEnabled);
        rightLeader.enableVoltageCompensation(DriveConstants.Electrical.kVoltageCompEnabled);
        rightFollower.enableVoltageCompensation(DriveConstants.Electrical.kVoltageCompEnabled);

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        rightLeader.setInverted(true);

        gyro.calibrate();
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftAppliedVolts = leftLeader.getMotorOutputVoltage();
        inputs.leftCurrentAmps = new double[] {leftLeader.getSupplyCurrent(), leftFollower.getSupplyCurrent()};
        inputs.rightAppliedVolts = rightLeader.getMotorOutputVoltage();
        inputs.rightCurrentAmps = new double[] {rightLeader.getSupplyCurrent(), rightFollower.getSupplyCurrent()};
        inputs.gyroYaw = gyro.getRotation2d();
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
