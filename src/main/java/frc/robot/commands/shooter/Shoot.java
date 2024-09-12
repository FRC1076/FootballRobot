package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {

    private final ShooterSubsystem m_ShooterSubsystem;
    private final DoubleSupplier speedSupplier;

    public Shoot(DoubleSupplier _speedSupplier, ShooterSubsystem ShooterSubsystem){
        m_ShooterSubsystem = ShooterSubsystem;
        speedSupplier = _speedSupplier;
        addRequirements(m_ShooterSubsystem);
        //System.out.println("Instatiating " + speedSupplier.getAsDouble()); //Debugging
    }

    @Override
    public void initialize(){
        m_ShooterSubsystem.setShooterSpeed(speedSupplier.getAsDouble() * ShooterConstants.kLeftMotorSpeedScalar,speedSupplier.getAsDouble() * ShooterConstants.kRightMotorSpeedScalar);
        //System.out.println("Shooting " + speedSupplier.getAsDouble()); //Debugging
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        m_ShooterSubsystem.stopShooterMotors();
        //System.out.println("End Shooting"); //Debugging
    }

}
