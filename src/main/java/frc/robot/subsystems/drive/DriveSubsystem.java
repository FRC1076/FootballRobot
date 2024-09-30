package frc.robot.subsystems.drive;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftFrontEncoderPort,DriveConstants.kLeftBackEncoderPort,DriveConstants.kLeftEncoderReversed);
    private final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightFrontEncoderPort, DriveConstants.kRightBackEncoderPort, DriveConstants.kRightEncoderReversed);

    private final DifferentialDrivePoseEstimator m_poseEstimator;

    private final Field2d m_field;

    private final LimelightPoseEstimator m_limelightEstimator;

    //New SysId Routine for characterizing the linear motion of the drive
    private final SysIdRoutine m_linearSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,null,null,
            (state) -> Logger.recordOutput("Drive/LinearSysIdTestState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> driveVolts(voltage.in(Volts),voltage.in(Volts)),
            null,
            this
        )
    );

    private final SysIdRoutine m_angularSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,null,null,
            (state) -> Logger.recordOutput("Drive/AngularSysIdTestState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> driveVolts(voltage.in(Volts),-voltage.in(Volts)),
            null,
            this
        )
    );

    public DriveSubsystem(DriveIO io) {
        this.io = io;

        // Config Encoders
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_leftEncoder.setMinRate(DriveConstants.kEncoderMinRate);
        m_leftEncoder.setSamplesToAverage(DriveConstants.kEncoderSamples);
        
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setMinRate(DriveConstants.kEncoderMinRate);
        m_rightEncoder.setSamplesToAverage(DriveConstants.kEncoderSamples);

        //Odometry
        m_poseEstimator = new DifferentialDrivePoseEstimator(
            new DifferentialDriveKinematics(DriveConstants.Physical.kTrackWidth), 
            inputs.gyroYaw, 
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
    }

    public void tankDrive(double moveSpeedLeft, double moveSpeedRight){
        System.out.println("Use arcade drive");
    }

    public void arcadeDrive(double moveSpeed, double turnSpeed){
        var speeds = DifferentialDrive.arcadeDriveIK(moveSpeed, turnSpeed, true);
        io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
    }

    public void driveVolts(double leftVolts, double rightVolts){
        io.setVoltage(leftVolts,rightVolts);
    }

    public void resetEncoders(){
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public void resetOdometry(Pose2d pose){
        resetEncoders();
        m_poseEstimator.resetPosition(inputs.gyroYaw, m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
    }

    @AutoLogOutput(key = "Drive/Pose")
    public Pose2d getPose(){
        return m_poseEstimator.getEstimatedPosition();
    }

    public double getAngle() {
        return inputs.gyroYaw.getDegrees();
    }

    public Rotation2d getRotation2d(){
        return inputs.gyroYaw;
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
        io.updateInputs(inputs);
        Logger.processInputs("Drive",inputs);
        m_poseEstimator.update(inputs.gyroYaw,m_leftEncoder.getDistance(),m_rightEncoder.getDistance());
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
}
