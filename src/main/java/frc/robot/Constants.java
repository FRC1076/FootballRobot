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
    public static final boolean kDriverInvertedControls = true;
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

    public static final double speedLimiter = 1.0; //Speed limiter, to keep robot from tipping over
    public static final double turnRateLimiter = 1.0; //Turn rate limiter, to keep robot from tipping over

    public static final double accelerationLimiter = 0.80; //Acceleration limiter, sets the time from neutral to full power, measured in seconds
  }

  public static class ShooterConstants {
    public static final int kLeftShooterMotorID = 0;
    public static final int kRightShooterMotorID = 0;

    public static final double kLeftShooterMotorSpeed = 1;
    public static final double kRightShooterMotorSpeed = 0.5;
  }
  
}
