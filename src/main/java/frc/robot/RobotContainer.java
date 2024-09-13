// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.ReducedDrive;
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


    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

    //Shuffleboard
    private final ShuffleboardTab ControlTab = Shuffleboard.getTab("Control");
    private final GenericEntry shooterSpeed = this.ControlTab
        .add("Shooter Speed",1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min",0,"max",1))
        .getEntry();

    //Drivetrain controls
    private final SendableChooser<String> driveModeChooser = new SendableChooser<>();
    private final ComplexWidget driveCommand = this.ControlTab
        .add("Drive Mode (Press A to confirm change)",driveModeChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(2,1);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
    private final CommandXboxController m_reducedController = 
        new CommandXboxController(OperatorConstants.kReducedControllerPort);

    //Runnable for changing the Drive mode
    private class changeDriveMode implements Runnable {
        @Override
        public void run(){
            switch (driveModeChooser.getSelected()){
                case "Arcade" -> CommandScheduler.getInstance().schedule(
                    new ArcadeDrive(
                        () -> MathUtil.applyDeadband(m_driverController.getLeftY() * (OperatorConstants.kDriverInvertedDriveControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
                        () -> MathUtil.applyDeadband(m_driverController.getRightX() * (OperatorConstants.kDriverInvertedTurnControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
                        m_robotDrive)
                    );
                case "Reduced" -> CommandScheduler.getInstance().schedule(
                    new ArcadeDrive(
                        () -> OperatorConstants.kReducedSpeedScalar * MathUtil.applyDeadband(m_reducedController.getLeftY() * (OperatorConstants.kDriverInvertedDriveControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
                        () -> OperatorConstants.kReducedSpeedScalar * MathUtil.applyDeadband(m_reducedController.getRightX() * (OperatorConstants.kDriverInvertedTurnControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
                        m_robotDrive)
                    );
                case "Disabled" -> CommandScheduler.getInstance().schedule(
                    new ArcadeDrive(
                        () -> 0.0, 
                        () -> 0.0, 
                        m_robotDrive)
                );
            }
        }
    }

    //Indexer motor

    private final PWMTalonSRX IndexMotor = new PWMTalonSRX(ShooterConstants.kIndexerMotorID);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        driveModeChooser.setDefaultOption("Arcade Drive","Arcade");
        driveModeChooser.addOption("Reduced Drive","Reduced");
        driveModeChooser.addOption("Drive Disabled (Shooting)", "Disabled");//For when we don't have an indexer

        // Configure the trigger bindings
        configureBindings();

        m_robotDrive.setDefaultCommand(new ArcadeDrive(
            () -> MathUtil.applyDeadband(m_driverController.getLeftY() * (OperatorConstants.kDriverInvertedDriveControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
            () -> MathUtil.applyDeadband(m_driverController.getRightX() * (OperatorConstants.kDriverInvertedTurnControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
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
        
        m_driverController.rightTrigger(0.5).whileTrue(new Shoot(
            () -> shooterSpeed.getDouble(0.5), 
            m_ShooterSubsystem));
        
            /*
        m_driverController.rightTrigger(0.5).whileTrue(new Shoot(
            () -> 0.5, 
            m_ShooterSubsystem));
        */
        //Configures Indexer Command
        m_driverController.leftTrigger(0.5).whileTrue(new StartEndCommand(
            () -> IndexMotor.set(1.0),
            () -> IndexMotor.stopMotor()
        ));
        
        //For testing
        /*
        m_driverController.leftTrigger(0.5).whileTrue(new StartEndCommand(
            () -> System.out.println("Beep"),
            () -> System.out.println("Boop")
        ));
        */

        //Toggles drive modes. (for testing only)
        /*
        m_driverController.x().toggleOnTrue(new ArcadeDrive(
            () -> MathUtil.applyDeadband(m_driverController.getLeftY() * (OperatorConstants.kDriverInvertedControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
            () -> MathUtil.applyDeadband(m_driverController.getRightX() * (OperatorConstants.kDriverInvertedControls ? -1 : 1), OperatorConstants.kDriverControllerDeadband),
            m_robotDrive
        ));
    */

        m_driverController.a().onTrue(new InstantCommand(new changeDriveMode()));
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
