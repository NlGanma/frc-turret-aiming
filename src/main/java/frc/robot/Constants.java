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
  }

  public static class FlywheelConstants {
    public static final int kFlywheelMotorCanId = 44;
    public static final double kFlywheelTargetRpm = 1000.0;
    public static final boolean kFlywheelInverted = false;
    public static final double kFlywheelKp = 0.0;
    public static final double kFlywheelKi = 0.0;
    public static final double kFlywheelKd = 0.0;
    public static final double kFlywheelKs = 0.0;
    public static final double kFlywheelKv = 0.12;
    public static final double kFlywheelKa = 0.0;
  }

  public static class HoodConstants {
    public static final int kHoodMotorCanId = 45;
    public static final double kMotorSprocketTeeth = 20.0;
    public static final double kHoodSprocketTeeth = 225.0;
    public static final double kMotorRotationsPerHoodRotation =
        kHoodSprocketTeeth / kMotorSprocketTeeth;
    public static final double kHoodMinAngleDegrees = 30.0;
    public static final double kHoodMaxAngleDegrees = 45.0;
    public static final double kHoodNearDistanceInches = 24.0;
    public static final double kHoodFarDistanceInches = 120.0;
    public static final boolean kHoodInverted = false;
    public static final double kHoodMaxVoltage = 4.0;
    public static final double kHoodKp = 24.0;
    public static final double kHoodKi = 0.0;
    public static final double kHoodKd = 0.0;
    public static final double kHoodKs = 0.0;
    public static final double kHoodKv = 0.0;
    public static final double kHoodKa = 0.0;
    public static final double kHoodStationaryTranslationMetersPerSecond = 0.05;
    public static final double kHoodStationaryRotationRadiansPerSecond = 0.05;
  }

  // This is the turret constants
  public static class TurretConstants {
    public static final int kTurretMotorCanId = 5;
    public static final double kMotorRotationsPerTurretRotation = 34.6;
    public static final double kClockwiseMotorDirection = 1.0;
    public static final double kPositionToleranceDegrees = 1.0;
    public static final double kAimPositionToleranceDegrees = 0.25;
    public static final double kTurretKp = 0.1285;
    public static final double kTurretKi = 0.0;
    public static final double kTurretKd = 0.0024;
    public static final double kFixedTargetXInches = 0.0;
    public static final double kFixedTargetYInches = 120.0;
    public static final double kTurretZeroFieldHeadingDegrees = 90.0;
    public static final double kTurretAimMaxOutput = 0.6;
    public static final double kTurretUnwindMaxOutput = 1.0;
    // Percent motor output added per turret degree/second of moving-target demand.
    public static final double kTurretAimVelocityFeedforward = 0.0005;
    public static final double kTurretMovingTargetVelocityThresholdDegreesPerSecond = 5.0;
    public static final double kTurretMinAngleDegrees = -360.0;
    public static final double kTurretMaxAngleDegrees = 360.0;
  }
}
