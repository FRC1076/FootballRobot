package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_leftLeader;
    private final WPI_TalonSRX m_leftFollower;
    private final WPI_TalonSRX m_rightLeader;
    private final WPI_TalonSRX m_rightFollower;
    private final MotorControllerGroup m_left;
    private final MotorControllerGroup m_right;

    private final DifferentialDrive m_differentialDrive;
    private final DifferentialDrivetrainSim m_driveSim;

    private final Encoder m_leftEncoder;
    private final Encoder m_rightEncoder;

    private final ADXRS450_Gyro m_gyro;

    private final DifferentialDriveOdometry m_odometry;

    private final EncoderSim m_leftEncoderSim;
    private final EncoderSim m_rightEncoderSim;
    private final ADXRS450_GyroSim m_gyroSim;

    private final Field2d m_field;

    public DriveSubsystem() {

        //Motors
        m_leftLeader = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);
        m_leftFollower = new WPI_TalonSRX(DriveConstants.kLeftBackMotorPort);
        m_rightLeader = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);
        m_rightFollower = new WPI_TalonSRX(DriveConstants.kRightBackMotorPort);
        m_left = new MotorControllerGroup(m_leftLeader, m_leftFollower);
        m_right = new MotorControllerGroup(m_rightLeader, m_rightFollower);

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
        m_gyro = new ADXRS450_Gyro();
        m_gyro.calibrate();

        //Odometry
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

        //field2d
        m_field = new Field2d();
        SmartDashboard.putData("Field",m_field);

        //Simulation
        m_leftEncoderSim = new EncoderSim(m_leftEncoder);
        m_rightEncoderSim = new EncoderSim(m_rightEncoder);
        m_gyroSim = new ADXRS450_GyroSim(m_gyro);
        m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),
            7.29,
            7.5,            60, 
            Units.inchesToMeters(3), 
            0.7112, 
            null); //Placeholder values
    }

    public void tankDrive(double moveSpeedLeft, double moveSpeedRight){
        m_differentialDrive.tankDrive(moveSpeedLeft,moveSpeedRight);
    }

    public void arcadeDrive(double moveSpeed, double turnSpeed){
        m_differentialDrive.arcadeDrive(-turnSpeed, moveSpeed);
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
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic(){
        m_driveSim.setInputs(m_leftLeader.get() * RobotController.getInputVoltage(), 
                            m_rightLeader.get() * RobotController.getInputVoltage());
        

        m_driveSim.update(0.02);

        m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
    }
}
