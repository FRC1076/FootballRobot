package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {

    private final ShooterSubsystem m_ShooterSubsystem;
    private final DoubleSupplier speedSupplier;

    public Shoot(DoubleSupplier _speedSupplier, ShooterSubsystem subsystem){
        m_ShooterSubsystem = subsystem;
        speedSupplier = _speedSupplier;
        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize(){
        m_ShooterSubsystem.setShooterSpeed(speedSupplier.getAsDouble() * ShooterConstants.kLeftMotorSpeedScalar,speedSupplier.getAsDouble() * ShooterConstants.kRightMotorSpeedScalar);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        m_ShooterSubsystem.stopShooterMotors();
    }

}
