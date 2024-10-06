package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ShooterConstants;

//TODO: Add support for Closed-loop speed control
public class ShooterIOCANSparkMax implements ShooterIO {
    private final CANSparkMax m_leftMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorID, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    public ShooterIOCANSparkMax(){
        //m_leftMotor.enableVoltageCompensation(12);
        //m_rightMotor.enableVoltageCompensation(12);
        m_leftMotor.setInverted(ShooterConstants.kLeftInverted);
        m_rightMotor.setInverted(ShooterConstants.kRightInverted);

        m_leftEncoder = m_leftMotor.getEncoder();
        m_rightEncoder = m_rightMotor.getEncoder();

        m_leftEncoder.setPositionConversionFactor(Math.PI * 2); // REV encoders take position in Rotations by default, this converts to Radians
        m_rightEncoder.setPositionConversionFactor(Math.PI * 2);

        m_leftEncoder.setVelocityConversionFactor((Math.PI * 2)/60); // REV encoders take velocity in RPM by default, this converts to Radians per Second
        m_rightEncoder.setVelocityConversionFactor((Math.PI * 2)/60);
    }

    /** sets voltage in open-loop control */
    @Override
    public void setVoltage(double leftVolts, double rightVolts){
        m_leftMotor.setVoltage(leftVolts);
        m_rightMotor.setVoltage(rightVolts);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        inputs.leftPositionRads = m_leftEncoder.getPosition();
        inputs.leftVelocityRadsPerSec = m_leftEncoder.getVelocity();
        inputs.leftAppliedVolts = m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage(); //the SparkMax API somehow doesn't have a built-in method to get output voltage
        inputs.leftCurrentAmps = m_leftMotor.getOutputCurrent();

        inputs.rightPositionRads = m_rightEncoder.getPosition();
        inputs.rightVelocityRadsPerSec = m_rightEncoder.getVelocity();
        inputs.rightAppliedVolts = m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage(); //the SparkMax API somehow doesn't have a built-in method to get output voltage
        inputs.rightCurrentAmps = m_rightMotor.getOutputCurrent();
    }


}
