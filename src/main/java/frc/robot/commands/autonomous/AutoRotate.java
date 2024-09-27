package frc.robot.commands.autonomous;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutonConstants.AutoRotationConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;

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
    private final boolean m_autoEnd; //Determines if the command ends automatically

    private GenericEntry pErrorEntry;
    private GenericEntry vErrorEntry;
    private GenericEntry pVariableEntry;
    private GenericEntry setpointEntry;
    private GenericEntry pidOutputEntry;
    
    /**
     * Constructs a new AutoRotate command
     * @param subsystem the drive subsystem
     * @param setpoint the desired final rotation of Chuck in degrees
     * @param processVariable the current measured rotation of Chuck in degrees
     */
    public AutoRotate(DoubleSupplier setpoint, DoubleSupplier processVariable, DriveSubsystem subsystem, boolean autoEnd){
        //System.out.println("Constructing autorotate");
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
        m_autoEnd = autoEnd;
        addRequirements(subsystem);
    }

    public AutoRotate(DoubleSupplier setpoint, DoubleSupplier processVariable, DriveSubsystem subsystem) {
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
        m_autoEnd = true;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.print("DriveSubsystem: Initializing autorotate");
        m_controller.reset();
        pErrorEntry = Shuffleboard.getTab("Autonomous").add("Autorotate: Position Error",m_controller.getPositionError()).getEntry();
        vErrorEntry = Shuffleboard.getTab("Autonomous").add("Autorotate: Velocity Error",m_controller.getVelocityError()).getEntry();
        pVariableEntry = Shuffleboard.getTab("Autonomous").add("Autorotate: Process Variable",m_processVariable.getAsDouble()).getEntry();
        setpointEntry = Shuffleboard.getTab("Autonomous").add("Autorotate: Setpoint",m_setpoint.getAsDouble()).getEntry();
        pidOutputEntry = Shuffleboard.getTab("Autonomous").add("Autorotate: PID Output",0).getEntry();
    }

    @Override
    public void execute() {
        double PIDOutput = m_controller.calculate(
            m_processVariable.getAsDouble(),
            m_setpoint.getAsDouble());
        m_driveSubsystem.arcadeDrive(0, PIDOutput);
        pErrorEntry.setDouble(m_controller.getPositionError());
        vErrorEntry.setDouble(m_controller.getVelocityError());
        pVariableEntry.setDouble(m_processVariable.getAsDouble());
        setpointEntry.setDouble(m_setpoint.getAsDouble());
        pidOutputEntry.setDouble(PIDOutput);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DriveSubsystem: Ending AutoRotate");
        pErrorEntry.close();
        vErrorEntry.close();
        pVariableEntry.close();
        setpointEntry.close();
        pidOutputEntry.close();
    }

    @Override
    public boolean isFinished() {
        return (m_autoEnd & m_controller.atSetpoint());
    }
}
