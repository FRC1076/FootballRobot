package frc.robot.commands.autonomous;


import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants.AutoRotationConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** A command to autonomously rotate towards a target and shoot */
public class RotateAndShoot extends Command {

    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shoot;
    private final DoubleSupplier m_setpoint;
    private final DoubleSupplier m_processVariable;
    private final BooleanSupplier m_acquiredTarget;
    private final BooleanSupplier m_shootingAuthorized;
    private final PIDController m_controller;

    /**
     * Constructs a new Autorotate command
     * 
     * @param setpoint the position of the desired target, relative to the robot
     * @param processVariable the robot's current orientaton
     * @param acquiredTarget whether or not the robot has acquired a target. This is to prevent the robot from ending the command prematurely if it loses track of the target
     * @param shootingAuthorized whether or not the robot is authorized to shoot, for safety
     * @param drive the drive subsystem
     * @param shoot the shoot subsystem
     */
    public RotateAndShoot(
        DoubleSupplier setpoint, 
        DoubleSupplier processVariable, 
        BooleanSupplier acquiredTarget, 
        BooleanSupplier shootingAuthorized, 
        DriveSubsystem drive, 
        ShooterSubsystem shoot
    ){
        m_drive = drive;
        m_shoot = shoot;
        m_setpoint = setpoint;
        m_processVariable = processVariable;
        m_controller = new PIDController(
            AutoRotationConstants.PIDCoefficients.kProportional, 
            AutoRotationConstants.PIDCoefficients.kIntegral, 
            AutoRotationConstants.PIDCoefficients.kDerivative);
        m_controller.enableContinuousInput(-180, 180);
        m_controller.setIntegratorRange(
            AutoRotationConstants.Integrator.kMin,
            AutoRotationConstants.Integrator.kMax);
        m_controller.setIZone(AutoRotationConstants.Integrator.kErrorThreshold);
        m_controller.setTolerance(
            AutoRotationConstants.Tolerance.kPosition,
            AutoRotationConstants.Tolerance.kVelocity);
        m_acquiredTarget = acquiredTarget;
        m_shootingAuthorized = shootingAuthorized;
        addRequirements(drive);
        addRequirements(shoot);
    }
}
