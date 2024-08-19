package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_leftLeader;
    private final WPI_TalonSRX m_leftFollower;
    private final WPI_TalonSRX m_rightLeader;
    private final WPI_TalonSRX m_rightFollower;
    private final MotorControllerGroup m_left;
    private final MotorControllerGroup m_right;

    private final DifferentialDrive m_differentialDrive;

    private final Encoder m_leftEncoder;
    private final Encoder m_rightEncoder;

    private final AnalogGyro m_gyro;

    private final DifferentialDriveOdometry m_odometry;

    public DriveSubsystem() {

        //Motors
        m_leftLeader = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);
        m_leftFollower = new WPI_TalonSRX(DriveConstants.kLeftBackMotorPort);
        m_rightLeader = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);
        m_rightFollower = new WPI_TalonSRX(DriveConstants.kRightBackMotorPort);
        m_left = new MotorControllerGroup(m_leftLeader, m_leftFollower);
        m_right = new MotorControllerGroup(m_rightLeader, m_rightFollower);
        //Pairing motors

        //m_rightLeader.setInverted(true);

        //Differential Drive
        m_differentialDrive = new DifferentialDrive(m_left,m_right);

        //Encoders
        m_leftEncoder = new Encoder(DriveConstants.kLeftFrontEncoderPort,DriveConstants.kLeftBackEncoderPort,DriveConstants.kLeftEncoderReversed);
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_leftEncoder.setMinRate(DriveConstants.kEncoderMinRate);
        m_leftEncoder.setSamplesToAverage(DriveConstants.kEncoderSamples);
        m_rightEncoder = new Encoder(DriveConstants.kRightFrontEncoderPort, DriveConstants.kRightBackEncoderPort, DriveConstants.kRightEncoderReversed);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setMinRate(DriveConstants.kEncoderMinRate);
        m_rightEncoder.setSamplesToAverage(DriveConstants.kEncoderSamples);

        //Gyro
        m_gyro = new AnalogGyro(DriveConstants.kGyroPort);
        m_gyro.calibrate();

        //Odometry
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }

    public void tankDrive(double moveSpeedLeft, double moveSpeedRight){
        m_differentialDrive.tankDrive(moveSpeedLeft,moveSpeedRight);
    }

    public void resetEncoders(){
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public void zeroHeading(){
        m_gyro.reset();
    }

    public void resetOdometry(Pose2d pose){
        resetEncoders();
        m_odometry.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public Rotation2d getRotation(){
        return m_gyro.getRotation2d();
    }

    @Override
    public void periodic() {
        //this method will be called once per scheduler run
        m_odometry.update(m_gyro.getRotation2d(),m_leftEncoder.getDistance(),m_rightEncoder.getDistance());
    }
}
