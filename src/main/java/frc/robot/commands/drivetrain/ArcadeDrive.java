package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * TODO: Write a javadoc for this class
 */
public class ArcadeDrive extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_speedStick;
    private final DoubleSupplier m_turnStick;
    private final String m_initMessage; //Initialization message, for identifying the drive mode in use

    public ArcadeDrive(DoubleSupplier speedStick, DoubleSupplier turnStick, String initMessage, DriveSubsystem subsystem) {
        m_driveSubsystem = subsystem;
        m_speedStick = speedStick;
        m_turnStick = turnStick;
        m_initMessage = initMessage;
        addRequirements(m_driveSubsystem);
    }

    public ArcadeDrive(DoubleSupplier speedStick, DoubleSupplier turnStick, DriveSubsystem subsystem) {
        //Overloaded constructor with default initialization message
        m_driveSubsystem = subsystem;
        m_speedStick = speedStick;
        m_turnStick = turnStick;
        m_initMessage = "Initializing Arcade Drive";
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        //For testing only
        System.out.println(m_initMessage);
    }

    @Override
    public void execute() {
        m_driveSubsystem.arcadeDrive(
            m_speedStick.getAsDouble(), 
            m_turnStick.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
