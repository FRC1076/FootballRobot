package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {

    private final ShooterSubsystem m_ShooterSubsystem;
    private final double speed;

    public Shoot(double _speed, ShooterSubsystem ShooterSubsystem){
        m_ShooterSubsystem = ShooterSubsystem;
        speed = _speed;
        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize(){
        m_ShooterSubsystem.setShooterSpeed(speed * ShooterConstants.kLeftMotorSpeedScalar,speed * ShooterConstants.kRightMotorSpeedScalar);
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        m_ShooterSubsystem.stopShooterMotors();
    }

}
