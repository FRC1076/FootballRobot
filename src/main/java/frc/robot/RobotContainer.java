// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.EnumSet;
import java.util.Map;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.autonomous.AutoRotate;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveIOTalonSRX;
import frc.robot.utils.Clacks;
import frc.robot.utils.limelight.LimelightHelpers;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(new DriveIOTalonSRX());
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

    //Shuffleboard & Networktables
    private final ShuffleboardTab ControlTab = Shuffleboard.getTab("Control");
    private final ShuffleboardTab AutonTab = Shuffleboard.getTab("Autonomous");
    private final SendableChooser<String> driveModeChooser = new SendableChooser<String>();
    private final SendableChooser<String> autonChooser = new SendableChooser<>();
    private final ComplexWidget driveCommand = this.ControlTab
        .add("Drive Mode",driveModeChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(2,1);
    private final ComplexWidget autonCommand = this.AutonTab
        .add("Autonomous Chooser",autonChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withSize(2,1);
    private StringSubscriber DMSub; //DO NOT CHANGE OUTSIDE CONFIGUREBINDINGS, CANNOT BE MADE FINAL BECAUSE REASONS
    private int DMValueListenerHandle; //DO NOT CHANGE OUTSIDE CONFIGUREBINDINGS, CANNOT BE MADE FINAL BECAUSE REASONS

    //Controls Shooter Speed
    private final GenericEntry shooterSpeed = this.ControlTab
        .add("Shooter Speed",1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min",0,"max",1))
        .getEntry();
    
    private final Clacks m_Trunk = new Clacks();

    //Controls drivetrain
    private final GenericEntry driveSpeed = this.ControlTab
        .add("Drive Speed Limit",1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min",0,"max",1))
        .getEntry();

    private final GenericEntry turnSpeed = this.ControlTab
        .add("Turn Speed Limit",1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min",0,"max",1))
        .getEntry();

    private final GenericEntry driveReversed = this.ControlTab
        .add("Drive Reversed",false)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OIConstants.kDriverControllerPort);
    
    //Factories for Drive Modes
    //TODO: put the command factories in their own class
    /**
     * Factory for Arcade Drive
     * @return a new ArcadeDrive() object, configured to act as a normal arcade drive
     */
    private ArcadeDrive ArcadeDriveFactory(){
        return new ArcadeDrive(
            () -> driveSpeed.getDouble(1.0) * MathUtil.applyDeadband(m_driverController.getLeftY() * (driveReversed.getBoolean(false) ? -1 : 1), OIConstants.kDriverControllerDeadband),
            () -> turnSpeed.getDouble(1.0) * MathUtil.applyDeadband(m_driverController.getRightX() * (OIConstants.kDriverInvertedTurnControls ? -1 : 1), OIConstants.kDriverControllerDeadband),
            "Drive Subsystem: Initializing Arcade Drive",
            m_robotDrive);
    }

    /**
     * Factory for clutch drive
     * @return a new ArcadeDrive() object, with all inputs reduced
     */
    private ArcadeDrive ClutchDriveFactory(){
        return new ArcadeDrive(
            () -> driveSpeed.getDouble(1.0) * DriveConstants.kClutchSpeed * MathUtil.applyDeadband(m_driverController.getLeftY() * (driveReversed.getBoolean(false) ? -1 : 1), OIConstants.kDriverControllerDeadband),
            () -> turnSpeed.getDouble(1.0) * DriveConstants.kClutchTurn * MathUtil.applyDeadband(m_driverController.getRightX() * (OIConstants.kDriverInvertedTurnControls ? -1 : 1), OIConstants.kDriverControllerDeadband),
            "Drive Subsystem: Initializing Clutch Drive",
            m_robotDrive);
    }

    /**
     * Factory for Disabled Drive
     * @return a new ArcadeDrive() object, configured with all inputs set to 0, effectively disabling the drive
     */
    private ArcadeDrive DisabledDriveFactory(){
        return new ArcadeDrive(
            () -> 0,
            () -> 0,
            "Drive Subsystem: Disabling Drive",
            m_robotDrive);
    }

    /**
     * Factory for Autonomous rotate 90 degrees test 
     * @return a new command, which will sequentially reset the gyroscope and then tell chuck to autonomously rotate 90 degrees
     */
    private SequentialCommandGroup AutoRotate90DegreesTestFactory(){
        double setpoint = m_robotDrive.getAngle() + 90.0;
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> System.out.println("DriveSubsystem: Turning 90 degrees")
            ),
            new AutoRotate(
                () -> setpoint,
                m_robotDrive::getAngle,
                m_robotDrive
            ),
            new InstantCommand(
                () -> changeDriveMode()
            )
        );
    }

    /** Factory for a command that autonomously rotates to an apriltag NOT TESTED*/
    private SequentialCommandGroup RotateToAprilTagFactory(){
        double setpoint = m_robotDrive.getAngle() - LimelightHelpers.getTX("limelight");
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> System.out.println("DriveSubsystem: Rotating to Apriltag")
            ),
            new AutoRotate(
                () -> setpoint,
                () -> m_robotDrive.getAngle(), 
                m_robotDrive
            ),
            new InstantCommand(
                () -> changeDriveMode()
            )
        );

    }

    /** Factory for a linear drivetrain characterization routine */
    private SequentialCommandGroup linearCharacterizationRoutine(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("DriveSubsystem: Performing linear characterization routine")),
            m_robotDrive.linearSysIdQuasistatic(Direction.kForward),
            m_robotDrive.linearSysIdQuasistatic(Direction.kReverse),
            m_robotDrive.linearSysIdDynamic(Direction.kForward),
            m_robotDrive.linearSysIdDynamic(Direction.kReverse)
        );
    }

    /** Factory for an angular drivetrain characterization routine */
    private SequentialCommandGroup angularCharacterizationRoutine(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println("DriveSubsystem: Performing angular characterization routine")),
            m_robotDrive.angularSysIdQuasistatic(Direction.kForward),
            m_robotDrive.angularSysIdQuasistatic(Direction.kReverse),
            m_robotDrive.angularSysIdDynamic(Direction.kForward),
            m_robotDrive.angularSysIdDynamic(Direction.kReverse)
        );
    }

    //Function for changing drive mode
    private void changeDriveMode(){
        //System.out.println("Changing Drive Mode"); //for debugging
        switch (driveModeChooser.getSelected()){
            case "Arcade" -> CommandScheduler.getInstance().schedule(ArcadeDriveFactory());
            case "Clutch" -> CommandScheduler.getInstance().schedule(ClutchDriveFactory());
            case "Reduced" -> System.out.println("Reduced Drive is currently WIP. Please do not use it.");
            case "Disabled" -> CommandScheduler.getInstance().schedule(DisabledDriveFactory());
        }
    }

    //Function for changing autonomous mode
    private void scheduleAutonomousCommand(){
        switch (autonChooser.getSelected()){
            case "Disabled" -> changeDriveMode();
            case "Rotate90Degrees" -> CommandScheduler.getInstance().schedule(AutoRotate90DegreesTestFactory());
            case "RotateToAprilTag" -> CommandScheduler.getInstance().schedule(RotateToAprilTagFactory());
            case "linearSysId" -> CommandScheduler.getInstance().schedule(linearCharacterizationRoutine());
            case "angularSysId" -> CommandScheduler.getInstance().schedule(angularCharacterizationRoutine());
        }
    }

    //Indexer motor

    private final WPI_TalonSRX IndexMotor = new WPI_TalonSRX(ShooterConstants.kIndexerMotorID);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        driveModeChooser.setDefaultOption("Arcade Drive","Arcade");
        driveModeChooser.addOption("Clutch Drive","Clutch");
        driveModeChooser.addOption("Reduced Drive (WIP)","Reduced");
        driveModeChooser.addOption("Drive Disabled", "Disabled");//For when we don't have an indexer

        autonChooser.setDefaultOption("Autonomous Disabled","Disabled");
        autonChooser.addOption("AutoRotate Test: Rotate 90 Degrees","Rotate90Degrees");
        autonChooser.addOption("Rotate to AprilTag", "RotateToAprilTag");
        autonChooser.addOption("Drivetrain Characterization Routine (Linear)", "linearSysId");
        autonChooser.addOption("Drivetrain Characterization Routine (Angular)", "angularSysId");

        // Configure the trigger bindings
        configureBindings();
        m_robotDrive.setDefaultCommand(ArcadeDriveFactory());
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
        //Also automatically disables drivetrain when shooting
        m_driverController.rightTrigger(0.5).whileTrue(
            new Shoot(
                () -> shooterSpeed.getDouble(0.5), 
                m_ShooterSubsystem
            )
            .raceWith(DisabledDriveFactory())
        )
        .negate().onTrue(new InstantCommand(() -> changeDriveMode()));
        
        //Configures Indexer Command
        m_driverController.leftTrigger(0.5).whileTrue(new StartEndCommand(
            () -> IndexMotor.set(1.0),
            () -> IndexMotor.stopMotor()
        ));
        
        //m_driverController.a().onTrue(new InstantCommand(() -> changeDriveMode()));

        //Configures Clutch binding (LB + RB)
        m_driverController.leftBumper().and(m_driverController.rightBumper())
            .whileTrue(ClutchDriveFactory())
            .negate().onTrue(new InstantCommand(() -> changeDriveMode()));

        //Configures emergency brake (b button)
        m_driverController.b()
            .whileTrue(DisabledDriveFactory())
            .negate().onTrue(new InstantCommand(() -> changeDriveMode()));
        
        //Configures autonomous mode switch (a button)
        m_driverController.a().onTrue(new InstantCommand(() -> scheduleAutonomousCommand()));

        //Configures Drive Mode listener (Not technically a binding, but serves the same purpose)
        NetworkTableInstance NTInst = NetworkTableInstance.getDefault();
        NetworkTable DMTable = NTInst.getTable("/Shuffleboard/Control/Drive Mode");
        DMSub = DMTable.getStringTopic("active").subscribe("Arcade");
        DMValueListenerHandle = NTInst.addListener(
            DMSub,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> changeDriveMode()
        );

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
