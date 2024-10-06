package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants.ShooterConstants;

//TODO: Add support for Closed-loop speed control
public class ShooterIOSparkMax implements ShooterIO {
    private final CANSparkMax m_leftMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorID, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor = new CANSparkMax(ShooterConstants.kRightShooterMotorID, MotorType.kBrushless);
    private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
    private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
    private final SparkPIDController m_leftController = m_leftMotor.getPIDController();
    private final SparkPIDController m_rightController = m_rightMotor.getPIDController();
    private static final double KP = 1.0; // TODO: MUST BE TUNED
    private static final double KI = 0.0; // TODO: MUST BE TUNED
    private static final double KD = 0.0; // TODO: MUST BE TUNED
    private static final double GEAR_RATIO = 1.0; // TODO: EDIT GEAR RATIO TO CHUCK'S SHOOTER GEAR RATIO

    public ShooterIOSparkMax(){
        m_leftMotor.enableVoltageCompensation(12.0);
        m_rightMotor.enableVoltageCompensation(12.0);
        m_leftMotor.setInverted(ShooterConstants.kLeftInverted);
        m_rightMotor.setInverted(ShooterConstants.kRightInverted);

        m_leftEncoder.setPositionConversionFactor(Math.PI * 2); // REV encoders take position in Rotations by default, this converts to Radians
        m_rightEncoder.setPositionConversionFactor(Math.PI * 2);

        m_leftEncoder.setVelocityConversionFactor((Math.PI * 2)/60); // REV encoders take velocity in RPM by default, this converts to Radians per Second
        m_rightEncoder.setVelocityConversionFactor((Math.PI * 2)/60);

        m_leftController.setP(KP);
        m_leftController.setI(KI);
        m_leftController.setD(KD);

        m_rightController.setP(KP);
        m_rightController.setI(KI);
        m_rightController.setD(KD);
    }

    /** sets voltage in open-loop control */
    @Override
    public void setVoltage(double leftVolts, double rightVolts){
        m_leftMotor.setVoltage(leftVolts);
        m_rightMotor.setVoltage(rightVolts);
    }

    /** Run closed loop at the specified velocity. */
    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
        m_leftController.setReference(
            Units.radiansPerSecondToRotationsPerMinute(leftRadPerSec),
            ControlType.kVelocity,
            0,
            leftFFVolts
        );

        m_rightController.setReference(
            Units.radiansPerSecondToRotationsPerMinute(rightRadPerSec),
            ControlType.kVelocity,
            0,
            rightFFVolts
        );
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
