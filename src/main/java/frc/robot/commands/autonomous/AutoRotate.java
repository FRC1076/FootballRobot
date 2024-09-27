package frc.robot.commands.autonomous;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutonConstants.AutoRotationConstants;


/**
 * A command that autonomously rotates Chuck towards an arbitrary setpoint using a PID controller
 * 
 * NOTE: Only rotates chuck, does not perform any translation
 */
public class AutoRotate extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final DoubleSupplier m_setpoint;
    private final DoubleSupplier m_processVariable;
    private final PIDController m_controller;

    /**
     * Constructs a new AutoRotate command
     * @param subsystem the drive subsystem
     * @param setpoint the desired final rotation of Chuck in degrees
     * @param processVariable the current measured rotation of Chuck in degrees
     */
    public AutoRotate(DoubleSupplier setpoint, DoubleSupplier processVariable, DriveSubsystem subsystem){
        System.out.println("Constructing autorotate");
        m_driveSubsystem = subsystem;
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
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.print("Initializing autorotate");
        m_controller.reset();
    }

    @Override
    public void execute() {
        double PIDOutput = m_controller.calculate(
            m_processVariable.getAsDouble(),
            m_setpoint.getAsDouble());
        System.out.println("Output=" + PIDOutput);
        System.out.println("PositionError=" + m_controller.getPositionError());
        System.out.println("VelocityError=" + m_controller.getVelocityError());
        System.out.println("Setpoint=" + m_controller.getSetpoint());
        m_driveSubsystem.arcadeDrive(0, PIDOutput);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending AutoRotate");
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}
