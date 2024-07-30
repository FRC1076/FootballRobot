package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class TankDrive extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_leftStick;
    private final DoubleSupplier m_rightStick;

    public TankDrive(DoubleSupplier leftStick, DoubleSupplier rightStick, DriveSubsystem subsystem){
        m_leftStick = leftStick;
        m_rightStick = rightStick;
        m_driveSubsystem = subsystem;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_driveSubsystem.tankDrive(m_leftStick.getAsDouble(), m_rightStick.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
