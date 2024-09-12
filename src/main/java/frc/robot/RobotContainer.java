// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    record SpeedWrapper (double speed) {}; //SendableChooser requires a reference, so a simple wrapper class is needed

    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

    //Shuffleboard
    private final ShuffleboardTab SmartDashboard = Shuffleboard.getTab("SmartDashboard");
    private final GenericEntry shooterSpeed = this.SmartDashboard
        .add("Shooter Speed",1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min",0,"max",1))
        .getEntry();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        m_robotDrive.setDefaultCommand(new ArcadeDrive(
            () -> MathUtil.applyDeadband(m_driverController.getLeftY() * (OperatorConstants.kDriverInvertedControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
            () -> MathUtil.applyDeadband(m_driverController.getRightX() * (OperatorConstants.kDriverInvertedControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
            m_robotDrive
        ));
    }

    /**
    * Use this method to define your trigger->command mappings. Triggers can be created via the
    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
    * predicate, or via the named factories in {@link
    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
    * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
    * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
    * joysticks}.
    */
    private void configureBindings() {
        //Configures shooter command
        m_driverController.leftTrigger(0.5).whileTrue(new Shoot(shooterSpeed.getDouble(0.0), m_ShooterSubsystem));
        //Toggles drive modes. (for testing only)
        /*
        m_driverController.x().toggleOnTrue(new ArcadeDrive(
            () -> MathUtil.applyDeadband(m_driverController.getLeftY() * (OperatorConstants.kDriverInvertedControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
            () -> MathUtil.applyDeadband(m_driverController.getRightX() * (OperatorConstants.kDriverInvertedControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
            m_robotDrive
        ));
    */

    /*
    //Toggles reduced drive mode (for testing only)
    m_driverController.x().toggleOnTrue(new ReducedDrive(
        () -> MathUtil.applyDeadband(m_driverController.getRightX() * (OperatorConstants.kDriverInvertedControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
        m_robotDrive
    ));
    */
    }

    private void configureShuffleboard() {

    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
     public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(m_exampleSubsystem);
    }
}
