package frc.robot.subsystems.mecanum;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.Constants.DriveConstants;

/** IO layer for running mecanum drive. Do not assume encoders are connected */
public class mecanumDriveIOTalonSRX implements mecanumDriveIO {
    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);
    private final WPI_TalonSRX leftBack = new WPI_TalonSRX(DriveConstants.kLeftBackMotorPort);
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);
    private final WPI_TalonSRX rightBack = new WPI_TalonSRX(DriveConstants.kRightBackMotorPort);
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    public mecanumDriveIOTalonSRX() {
        var config = new TalonSRXConfiguration();
        config.peakCurrentLimit = DriveConstants.Electrical.kPeakCurrentLimit;
        config.peakCurrentDuration = DriveConstants.Electrical.kPeakCurrentDuration;
        config.continuousCurrentLimit = DriveConstants.Electrical.kContinuousCurrentLimit;
        config.voltageCompSaturation = DriveConstants.Electrical.kVoltageComp; //"Full output" will scale to 12 volts, regardless of actual battery voltage level
        
        leftFront.configAllSettings(config);
        leftBack.configAllSettings(config);
        rightFront.configAllSettings(config);
        rightBack.configAllSettings(config);
        leftFront.enableVoltageCompensation(DriveConstants.Electrical.kVoltageCompEnabled);
        leftBack.enableVoltageCompensation(DriveConstants.Electrical.kVoltageCompEnabled);
        rightFront.enableVoltageCompensation(DriveConstants.Electrical.kVoltageCompEnabled);
        rightBack.enableVoltageCompensation(DriveConstants.Electrical.kVoltageCompEnabled);

        gyro.calibrate();
        System.out.println("Gyro calibrated");
    }

    @Override
    public void updateInputs(mecanumDriveIOInputs inputs){
        inputs.leftFrontAppliedVolts = leftFront.getMotorOutputVoltage();
        inputs.leftFrontCurrentAmps = leftFront.getSupplyCurrent();
        inputs.leftBackAppliedVolts = leftBack.getMotorOutputVoltage();
        inputs.leftBackCurrentAmps = leftBack.getSupplyCurrent();
        inputs.rightFrontAppliedVolts = rightFront.getMotorOutputVoltage();
        inputs.rightFrontCurrentAmps = rightFront.getSupplyCurrent();
        inputs.rightBackAppliedVolts = rightBack.getMotorOutputVoltage();
        inputs.rightBackCurrentAmps = rightBack.getSupplyCurrent();

        inputs.gyroYaw = gyro.getRotation2d();
        inputs.gyroRate = gyro.getRate();
    }

    @Override
    public void setVoltage(double leftFrontVolts, double leftBackVolts, double rightFrontVolts, double rightBackVolts) {
        leftFront.set(ControlMode.PercentOutput,leftFrontVolts/12.0);
        leftBack.set(ControlMode.PercentOutput,leftBackVolts/12.0);
        rightFront.set(ControlMode.PercentOutput,rightFrontVolts/12.0);
        rightBack.set(ControlMode.PercentOutput,rightBackVolts/12.0);
    }
}
