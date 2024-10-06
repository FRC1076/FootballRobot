package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }
    
    public void setShooterSpeed(double leftMotorSpeed, double rightMotorSpeed) {
        io.setVoltage(leftMotorSpeed * 12, rightMotorSpeed * 12);
    }

    /**
     * Stops the motors
     */
    public void stopShooterMotors() {
        io.setVoltage(0, 0);
    }
    

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter",inputs);
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

}