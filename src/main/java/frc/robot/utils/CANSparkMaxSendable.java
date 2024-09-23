package frc.robot.utils;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A wrapper class for CANSparkMax that implements sendables
 */
public class CANSparkMaxSendable extends CANSparkMax implements Sendable {
    public CANSparkMaxSendable(int deviceID, MotorType type) {
        super(deviceID,type);
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }
}