package frc.robot.subsystems.drive;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.limelight.LimelightHelpers;
import frc.robot.utils.limelight.LimelightPoseEstimator;

public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);
    private final WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(DriveConstants.kLeftBackMotorPort);;
    private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);
    private final WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(DriveConstants.kRightBackMotorPort);

    private final DifferentialDrive m_differentialDrive;
    private final DifferentialDrivetrainSim m_driveSim;

    private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftFrontEncoderPort,DriveConstants.kLeftBackEncoderPort,DriveConstants.kLeftEncoderReversed);;
    private final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightFrontEncoderPort, DriveConstants.kRightBackEncoderPort, DriveConstants.kRightEncoderReversed);;

    private final ADXRS450_Gyro m_gyro;

    private final DifferentialDrivePoseEstimator m_poseEstimator;

    private final EncoderSim m_leftEncoderSim;
    private final EncoderSim m_rightEncoderSim;
    private final ADXRS450_GyroSim m_gyroSim;

    private final Field2d m_field;

    private final LimelightPoseEstimator m_limelightEstimator;

    //New SysId Routine for characterizing the linear motion of the drive
    private final SysIdRoutine m_linearSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,null,null,
            (state) -> Logger.recordOutput("DrivetrainLinearSysIdTestState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            voltage -> {
                m_leftLeader.setVoltage(voltage.in(Volts));
                m_rightLeader.setVoltage(voltage.in(Volts));
            },
            null,
            this
        )
    );

    private final SysIdRoutine m_angularSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,null,null,
            (state) -> Logger.recordOutput("DrivetrainAngularSysIdTestState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            voltage -> {
                m_leftLeader.setVoltage(voltage.in(Volts));
                m_rightLeader.setVoltage(-voltage.in(Volts));
            },
            null,
            this
        )
    );

    public DriveSubsystem() {

        //Motor Config
        m_leftLeader.configOpenloopRamp(DriveConstants.kAccelerationLimiter);
        m_leftLeader.setNeutralMode(NeutralMode.Brake);

        m_leftFollower.configOpenloopRamp(DriveConstants.kAccelerationLimiter);
        m_leftFollower.setNeutralMode(NeutralMode.Brake);

        m_rightLeader.configOpenloopRamp(DriveConstants.kAccelerationLimiter);
        m_rightLeader.setNeutralMode(NeutralMode.Brake);

        m_rightFollower.configOpenloopRamp(DriveConstants.kAccelerationLimiter);
        m_rightFollower.setNeutralMode(NeutralMode.Brake);

        m_leftFollower.follow(m_leftLeader);
        m_rightFollower.follow(m_rightLeader);

        SupplyCurrentLimitConfiguration currentLimitConfig = new SupplyCurrentLimitConfiguration(
            DriveConstants.Electrical.kCurrentLimitEnabled,
            DriveConstants.Electrical.kCurrentLimit,
            DriveConstants.Electrical.kCurrentThreshold,
            DriveConstants.Electrical.kCurrentLimitTriggerTime
        );

        m_leftLeader.configSupplyCurrentLimit(currentLimitConfig);
        m_rightLeader.configSupplyCurrentLimit(currentLimitConfig);
        m_leftFollower.configSupplyCurrentLimit(currentLimitConfig);
        m_rightFollower.configSupplyCurrentLimit(currentLimitConfig);
        
        m_rightLeader.setInverted(true);

        //Differential Drive
        m_differentialDrive = new DifferentialDrive(m_leftLeader,m_rightLeader);
        addChild("Drivetrain", m_differentialDrive);

        // Config Encoders
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_leftEncoder.setMinRate(DriveConstants.kEncoderMinRate);
        m_leftEncoder.setSamplesToAverage(DriveConstants.kEncoderSamples);
        
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setMinRate(DriveConstants.kEncoderMinRate);
        m_rightEncoder.setSamplesToAverage(DriveConstants.kEncoderSamples);

        //Gyro
        m_gyro = new ADXRS450_Gyro();
        m_gyro.calibrate();
        addChild("Gyro", m_gyro);

        //Odometry
        m_poseEstimator = new DifferentialDrivePoseEstimator(
            new DifferentialDriveKinematics(DriveConstants.Physical.kTrackWidth), 
            m_gyro.getRotation2d(), 
            m_leftEncoder.getDistance(), 
            m_rightEncoder.getDistance(),
            new Pose2d());
        m_limelightEstimator = new LimelightPoseEstimator("limelight", new Transform2d(
            new Translation2d(
                VisionConstants.TransformConstants.kTransformX,
                VisionConstants.TransformConstants.kTransformY
            ),
            new Rotation2d(VisionConstants.TransformConstants.kTransformRot)));

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
        m_differentialDrive.arcadeDrive(moveSpeed, turnSpeed);
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
        m_poseEstimator.resetPosition(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
    }

    public Pose2d getPose(){
        return m_poseEstimator.getEstimatedPosition();
    }

    public double getAngle() {
        return m_gyro.getAngle();
    }

    public Rotation2d getRotation2d(){
        return m_gyro.getRotation2d();
    }

    // System Identification routine factories

    /** A factory for a quasistatic system identification routine for the drivetrain, to test its linear motion */
    public Command linearSysIdQuasistatic(SysIdRoutine.Direction direction){
        return m_linearSysIdRoutine.quasistatic(direction);
    }
    /** A factory for a dynamic system identification routine for the drivetrain, to test its linear motion */
    public Command linearSysIdDynamic(SysIdRoutine.Direction direction){
        return m_linearSysIdRoutine.dynamic(direction);
    }
    /** A factory for a quasistatic system identification routine for the drivetrain, to test its angular motion */
    public Command angularSysIdQuasistatic(SysIdRoutine.Direction direction){
        return m_angularSysIdRoutine.quasistatic(direction);
    }
    /** A factory for a dynamic system identification routine for the drivetrain, to test its angular motion */
    public Command angularSysIdDynamic(SysIdRoutine.Direction direction){
        return m_angularSysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {
        //this method will be called once per scheduler run
        m_poseEstimator.update(m_gyro.getRotation2d(),m_leftEncoder.getDistance(),m_rightEncoder.getDistance());
        Optional<LimelightHelpers.PoseEstimate> limelightPose = m_limelightEstimator.getPose();
        if(limelightPose.isPresent()){
            m_poseEstimator.addVisionMeasurement(limelightPose.get().pose, limelightPose.get().timestampSeconds);
            System.out.println("TAG DISTANCE AT " + limelightPose.get().timestampSeconds + ":");
            //System.out.println(LimelightHelpers.getJSONDump(Constants.VisionConstants.limelight1));
            System.out.println(limelightPose.get().avgTagDist);
        }
        //System.out.println(m_gyro.getAngle());
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
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
        m_gyroSim.setAngle(m_driveSim.getHeading().getDegrees());
    }
}
