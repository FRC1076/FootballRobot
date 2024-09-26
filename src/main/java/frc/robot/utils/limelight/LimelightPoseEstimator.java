package frc.robot.utils.limelight;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.limelight.LimelightHelpers.PoseEstimate;

public class LimelightPoseEstimator {
    private final String limelightName; // name of the limelight
    private final Transform2d offset; // a shift to apply if the limelight is consistently offset from the center of the robot
    
    /**
     * A dataclass for storing limelight pose and timestamp
     */
    public record LimelightPose(Pose2d pose, double timestamp){
        public Pose2d getPose(){
            return pose;
        }

        public double getTimestamp(){
            return timestamp;
        }
    };

    /**
     * Creates a new LimelightPoseEstimator.
     * 
     * LimelightPoseEstimator is a helper class that helps LimelightHelper
     * 
     * @param limelightName is the name of the limelight
     * @param offset is a Transform2d to correct any offset
     */
    public LimelightPoseEstimator(String _limelightString, Transform2d _offset){
        this.limelightName = _limelightString;
        this.offset = _offset;
    }

    /**
     * @return the total latency of the limelight (ms)
     */
    public double getLatency(){
        return (LimelightHelpers.getLatency_Capture(limelightName) + LimelightHelpers.getLatency_Pipeline(limelightName));
    }

    /**
     * @return the total latency of the limelight (in seconds)
     */
    public double getLatencyS(){
        return getLatency() * 0.001;
    }
    /**
     * @return pose2d from limelight
     */
    private PoseEstimate getPoseRaw(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    /**
     * returns pose from the limelight
     * @return pose, or empty
     */
    public Optional<PoseEstimate> getPose(){
        PoseEstimate limelightPose = getPoseRaw();
        if (limelightPose.pose.equals(new Pose2d()) || LimelightHelpers.getTA(limelightName) < VisionConstants.kTargetAreaThreshold){
            return Optional.empty();
        } else {
            limelightPose.pose = limelightPose.pose.transformBy(offset);
            return Optional.of(limelightPose);
        }
    }

    /**
     * @return timestamp of capture
     */
    public double getCaptureTimestamp(){
        return Timer.getFPGATimestamp() - getLatencyS();
    }
}