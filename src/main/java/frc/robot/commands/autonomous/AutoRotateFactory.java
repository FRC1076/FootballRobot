package frc.robot.commands.autonomous;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.drive.DriveSubsystem;
//Factory for autorotate commands
public final class AutoRotateFactory {

    /**Factory for an autorotate command that rotates to a fixed setpoint */
    public static AutoRotate FixedSetpoint(double setpoint, DoubleSupplier processVariable, DriveSubsystem subsystem){
        return new AutoRotate(() -> setpoint, processVariable, subsystem);
    }
}
