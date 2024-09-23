// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
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
  }

  public static class ShooterConstants {
    public static final int kLeftShooterMotorID = 6; //Placeholder
    public static final int kRightShooterMotorID = 1; //Placeholder

    public static final double kLeftMotorSpeedScalar = 1.0;
    public static final double kRightMotorSpeedScalar = 0.5;

    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;

    public static final int kIndexerMotorID = 3; //Placeholder

    public static class Electrical {
        public final double kCurrentLimit = 40; //Current limit in amperes
        public final double kCurrentThreshold = 60; //The threshold at which the current limit is activated
        public final double kCurrentLimitTriggerTime = 
    }
  }
  
}
