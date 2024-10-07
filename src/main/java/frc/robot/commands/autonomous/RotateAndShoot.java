package frc.robot.commands.autonomous;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants.AutoRotationConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** A command to autonomously rotate towards a target and shoot */
public class RotateAndShoot extends Command {

    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shoot;
    private final DoubleSupplier m_setpoint;
    private final DoubleSupplier m_processVariable;
    private final BooleanSupplier m_acquiredTarget;
    private final BooleanSupplier m_shootingAuthorized;
    private Boolean commandEnded = false;
    private final PIDController m_controller;
    private final Timer m_timer = new Timer();

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
            AutoRotationConstants.ConTolerance.kPosition,
            AutoRotationConstants.ConTolerance.kVelocity);
        m_acquiredTarget = acquiredTarget;
        m_shootingAuthorized = shootingAuthorized;
        addRequirements(drive);
        addRequirements(shoot);
    }

    @Override
    public void initialize() {
        System.out.println("DriveSubsystem: Initializing RotateAndShoot");
        System.out.println("ShooterSubsystem: Initializing RotateAndShoot");
        System.out.println("DriveSubsystem: KILL ALL HUMANS");
        System.out.println("ShooterSubsystem: KILL ALL HUMANS");
        m_controller.reset();
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    @Override
    public void execute() {
        //For debugging
        /*
        System.out.println("At Setpoint: " + m_controller.atSetpoint());
        System.out.println("Authorized: " + m_shootingAuthorized.getAsBoolean());
        System.out.println("Target Acquired: " + m_acquiredTarget.getAsBoolean());
        */
        double PIDOutput = m_controller.calculate(
            m_processVariable.getAsDouble(),
            m_setpoint.getAsDouble());
        m_drive.arcadeDriveNoSquare(0, PIDOutput);
        if (m_controller.atSetpoint() & m_acquiredTarget.getAsBoolean() & m_shootingAuthorized.getAsBoolean()) {
            m_shoot.setShooterSpeed(
                ShooterConstants.kLeftMotorSpeedScalar,
                ShooterConstants.kRightMotorSpeedScalar
            );
            m_timer.start();
            if (m_timer.advanceIfElapsed(1.5)) {
                //TODO: activate indexer
                commandEnded = true;
            }
        } else {
            m_shoot.stopShooterMotors();
            m_timer.stop();
            m_timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stopMotors();
        m_shoot.stopShooterMotors();
        System.out.println("DriveSubsystem: Ending RotateAndShoot");
        System.out.println("ShooterSubsystem: Ending RotateAndShoot");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    @Override
    public boolean isFinished() {
        return commandEnded;
    }
}
