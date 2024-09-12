package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ReducedDrive extends Command {

    //ReducedDrive is a reduced-functionality drive system, which only allows turning
    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_turnStick;

    public ReducedDrive(DoubleSupplier turnStick, DriveSubsystem subsystem){
        m_driveSubsystem = subsystem;
        m_turnStick = turnStick;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        //For testing only
        System.out.println("Reduced Drive");
    }

    @Override
    public void execute() {
        m_driveSubsystem.arcadeDrive(
            0.0, 
            m_turnStick.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
