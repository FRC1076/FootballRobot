package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_speedStick;
    private final DoubleSupplier m_turnStick;

    public ArcadeDrive(DoubleSupplier speedStick, DoubleSupplier turnStick, DriveSubsystem subsystem) {
        m_driveSubsystem = subsystem;
        m_speedStick = speedStick;
        m_turnStick = turnStick;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        //For testing only
        //System.out.println("Arcade Drive");
    }

    @Override
    public void execute() {
        m_driveSubsystem.arcadeDrive(
            m_speedStick.getAsDouble() * DriveConstants.kSpeedLimiter, 
            m_turnStick.getAsDouble() * DriveConstants.kTurnRateLimiter);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
