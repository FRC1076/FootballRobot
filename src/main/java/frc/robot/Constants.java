// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Akit {

        /**
         * Determines the mode that AdvantageKit will run in.
         * <ul>
         * <li>0 = Running on a real robot</li>
         * <li>1 = Running on a simulator</li>
         * <li>2 = Replaying from a log file</li>
         * </ul>
         * currentMode's value can be changed as needed in the Constants.java file
         * before compile time. Please ensure that currentMode is set to 0 (real)
         * before pushing any changes to github.
         */
        public static final int currentMode = 0;
    }
  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerDeadband = 0.1;
    public static final boolean kDriverInvertedDriveControls = false;
    public static final boolean kDriverInvertedTurnControls = true;

    public static final int kReducedControllerPort = 1;
    public static final double kReducedSpeedScalar = 0.2; //Speed scalar for when the robot is operating in reduced control mode. 0.3, for example, reduces all controller inputs to 30%
  }

  //For the time being, all of these values are placeholders
  public static class DriveConstants {
    public static final int kLeftFrontMotorPort = 7;
    public static final int kLeftBackMotorPort = 2;
    public static final int kRightFrontMotorPort = 5;
    public static final int kRightBackMotorPort = 3;

    public static final int kLeftFrontEncoderPort = 1;
    public static final int kLeftBackEncoderPort = 2;
    public static final boolean kLeftEncoderReversed = false;
    public static final int kRightFrontEncoderPort = 3;
    public static final int kRightBackEncoderPort = 4;
    public static final boolean kRightEncoderReversed = true;

    public static final double kEncoderDistancePerPulse = 4.0/256.0;
    public static final double kEncoderMinRate = 10;
    public static final int kEncoderSamples = 5;

    public static final int kGyroPort = 0;
    public static final boolean kGyroReversed = false;

    public static final double kClutchSpeed = 0.4; //Speed limiter for clutch
    public static final double kClutchTurn = 0.4; //Turn rate limiter for clutch

    public static final double kAccelerationLimiter = 0.00; //Acceleration limiter, sets the time from neutral to full power, measured in seconds

    public static class Electrical {
        public static final int kContinuousCurrentLimit = 40; //Current limit in amperes PLACEHOLDER
        public static final int kPeakCurrentLimit = 60; //The threshold at which the current limit is activated PLACEHOLDER
        public static final int kPeakCurrentDuration = 250; //The time in milliseconds that the current needs to be over the threshold before the limiter is activated PLACEHOLDER
        public static final boolean kVoltageCompEnabled = true; // whether or not voltage compensation is activated
        public static final double kVoltageComp = 12.0; // Voltage compensation
    }

    public static class Physical {
        public static final double kTrackWidth = 0.5; //Width of the drivetrain, in meters PLACEHOLDER
        public static final double kWheelBase = 0.5; //Length from front wheel to back wheel, in meters PLACEHOLDER
    }
  }

  public static class ShooterConstants {
    public static final int kLeftShooterMotorID = 6;
    public static final int kRightShooterMotorID = 1;

    public static final double kLeftMotorSpeedScalar = 1.0;
    public static final double kRightMotorSpeedScalar = 0.5;

    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;

    public static final int kIndexerMotorID = 4;

    public static class Electrical {
        public static final double kCurrentLimit = 40; //Current limit in amperes PLACEHOLDER
    }
  }

  public static class BallisticsConstants {
    public static final double kShooterAngle = 35; //The shooter's angle, in degrees PLACEHOLDER
    public static final double kGravity = 9.81; //Gravitational acceleration, in m/s/s
    public static final double kShooterHeight = 0.05; //The shooter's height above the ground, in meters PLACEHOLDER
  }

  public static class VisionConstants {
    public static final double kTargetAreaThreshold = 0.01;

    public static class TransformConstants {
        public static final double kTransformX = 0; //position of the limelight in meters relative to the center of the robot PLACEHOLDER
        public static final double kTransformY = 0; //position of the limelight in meters relative to the center of the robot PLACEHOLDER
        public static final double kTransformRot = 0; //position of the limelight in meters relative to the center of the robot PLACEHOLDER
    }   
  }

  public static class AutonConstants {

    /**
     * PID Constants for Autonomous Rotation
     */
    public static class AutoRotationConstants {

        //PID coefficients
        public static class PIDCoefficients {
            public static final double kProportional = 0.015;
            public static final double kIntegral = 0.001;
            public static final double kDerivative = 0.00000;
        }

        //Target tolerance
        public static class Tolerance {
            public static final double kPosition = 3; //Position tolerance (in degrees) (NOTE: Position refers to the process variable)
            public static final double kVelocity = 5; //Velocity tolerance (in degrees per second) (NOTE: Velocity refers to the derivative of the process variable)
        }

        //Integrator constants
        public static class Integrator {
            public static final double kErrorThreshold = 20; //Disables the integrator once the absolute value of the error crosses this threshold. Set to Double.POSITIVE_INFINITY to disable
            public static final double kMin = -10; //Minimum value of the integrator term. if the integrator term is below this value, then it is no longer multiplied by the Integrator gain
            public static final double kMax = 10; //Maximum value of the integrator term. if the integrator term is above this value, then it is no longer multiplied by the Integrator gain
        }
    }
  }
}
