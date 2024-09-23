package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.utils.CANSparkMaxSendable;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    public final CANSparkMaxSendable m_leftShooterMotor;
    public final CANSparkMaxSendable m_rightShooterMotor;

    public ShooterSubsystem() {
        m_leftShooterMotor = new CANSparkMaxSendable(ShooterConstants.kLeftShooterMotorID, CANSparkMax.MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMaxSendable(ShooterConstants.kRightShooterMotorID, CANSparkMax.MotorType.kBrushless);
        m_leftShooterMotor.setInverted(ShooterConstants.kLeftInverted);
        m_rightShooterMotor.setInverted(ShooterConstants.kRightInverted);
        addChild("Left Shooter Motor",m_leftShooterMotor);
        addChild("Right Shooter Motor",m_rightShooterMotor);
    }
    
    public void setShooterSpeed(double leftMotorSpeed, double rightMotorSpeed) {
        m_leftShooterMotor.set(leftMotorSpeed);
        m_rightShooterMotor.set(rightMotorSpeed); 
    }

    /**
     * @return the left motor's power, on a scale from -1.0 to 1.0
     */
    public double getLeftPower() {
        return m_leftShooterMotor.get();
    }

    
    /**
     * @return the right motor's power, on a scale from -1.0 to 1.0
     */
    public double getRightPower() {
        return m_rightShooterMotor.get();
    }

    /**
     * Stops the motors
     */
    public void stopShooterMotors() {
        m_leftShooterMotor.set(0);
        m_rightShooterMotor.set(0);
    }
    

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

}