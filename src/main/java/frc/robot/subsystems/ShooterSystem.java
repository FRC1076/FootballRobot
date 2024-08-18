package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ShooterConstants;

public class ShooterSystem extends SubsystemBase {

    public final CANSparkMax m_leftShooterMotor;
    public final CANSparkMax m_rightShooterMotor;

    public ShooterSystem() {
        m_leftShooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorID, CANSparkMax.MotorType.kBrushless);
        m_rightShooterMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorID, CANSparkMax.MotorType.kBrushless);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

}
