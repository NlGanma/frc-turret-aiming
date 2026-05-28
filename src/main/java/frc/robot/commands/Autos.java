// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public final class Autos {
  private static final double kSquareSideMeters = Units.inchesToMeters(40.0);
  private static final double kSquarePositionToleranceMeters = Units.inchesToMeters(1.0);
  private static final double kSquareMaxSpeedMetersPerSecond = Units.inchesToMeters(24.0);
  private static final double kSquareMaxAngularRateRadiansPerSecond = Math.toRadians(90.0);
  private static final double kSquareTranslationKp = 1.8;
  private static final double kSquareHeadingKp = 3.0;
  private static final double kSquareHeadingToleranceRadians = Math.toRadians(2.0);
  private static final double kSquareWaypointTimeoutSeconds = 3.5;

  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command driveFortyInchSquare(CommandSwerveDrivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              drivetrain.resetPose(new Pose2d());
              SmartDashboard.putString("SquareTest/Waypoint", "Reset");
            },
            drivetrain),
        driveToFieldPosition(drivetrain, kSquareSideMeters, 0.0, "+X 40 in"),
        driveToFieldPosition(drivetrain, kSquareSideMeters, kSquareSideMeters, "+Y 40 in"),
        driveToFieldPosition(drivetrain, 0.0, kSquareSideMeters, "-X 40 in"),
        driveToFieldPosition(drivetrain, 0.0, 0.0, "-Y 40 in"),
        Commands.runOnce(
            () -> {
              drivetrain.setControl(new SwerveRequest.Idle());
              SmartDashboard.putString("SquareTest/Waypoint", "Done");
            },
            drivetrain));
  }

  public static Command turnTurretNinetyDegreesAndBack(TurretSubsystem turretSubsystem) {
    return Commands.sequence(
        turretSubsystem.turnTurretDegrees(90.0, 1, 0.6),
        turretSubsystem.turnTurretDegrees(0.0, 1, 0.6));
  }

  private static Command driveToFieldPosition(
      CommandSwerveDrivetrain drivetrain,
      double targetXMeters,
      double targetYMeters,
      String waypointName) {
    PIDController xController = new PIDController(kSquareTranslationKp, 0.0, 0.0);
    PIDController yController = new PIDController(kSquareTranslationKp, 0.0, 0.0);
    PIDController headingController = new PIDController(kSquareHeadingKp, 0.0, 0.0);
    SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    xController.setTolerance(kSquarePositionToleranceMeters);
    yController.setTolerance(kSquarePositionToleranceMeters);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    headingController.setTolerance(kSquareHeadingToleranceRadians);

    return Commands.runEnd(
            () -> {
              Pose2d pose = drivetrain.getDrivePose2d();
              double velocityX =
                  clamp(
                      xController.calculate(pose.getX(), targetXMeters),
                      -kSquareMaxSpeedMetersPerSecond,
                      kSquareMaxSpeedMetersPerSecond);
              double velocityY =
                  clamp(
                      yController.calculate(pose.getY(), targetYMeters),
                      -kSquareMaxSpeedMetersPerSecond,
                      kSquareMaxSpeedMetersPerSecond);
              double rotationalRate =
                  clamp(
                      headingController.calculate(pose.getRotation().getRadians(), 0.0),
                      -kSquareMaxAngularRateRadiansPerSecond,
                      kSquareMaxAngularRateRadiansPerSecond);

              drivetrain.setControl(
                  driveRequest
                      .withVelocityX(velocityX)
                      .withVelocityY(velocityY)
                      .withRotationalRate(rotationalRate));
              publishSquareTelemetry(waypointName, targetXMeters, targetYMeters, pose);
            },
            () -> drivetrain.setControl(new SwerveRequest.Idle()),
            drivetrain)
        .beforeStarting(
            () -> {
              xController.reset();
              yController.reset();
              headingController.reset();
              SmartDashboard.putString("SquareTest/Waypoint", waypointName);
            })
        .until(
            () -> atFieldPosition(drivetrain.getDrivePose2d(), targetXMeters, targetYMeters))
        .withTimeout(kSquareWaypointTimeoutSeconds);
  }

  private static boolean atFieldPosition(
      Pose2d pose, double targetXMeters, double targetYMeters) {
    return Math.abs(targetXMeters - pose.getX()) <= kSquarePositionToleranceMeters
        && Math.abs(targetYMeters - pose.getY()) <= kSquarePositionToleranceMeters
        && Math.abs(normalizeRadians(pose.getRotation().getRadians()))
            <= kSquareHeadingToleranceRadians;
  }

  private static void publishSquareTelemetry(
      String waypointName, double targetXMeters, double targetYMeters, Pose2d pose) {
    SmartDashboard.putString("SquareTest/Waypoint", waypointName);
    SmartDashboard.putNumber("SquareTest/TargetXInches", Units.metersToInches(targetXMeters));
    SmartDashboard.putNumber("SquareTest/TargetYInches", Units.metersToInches(targetYMeters));
    SmartDashboard.putNumber("SquareTest/PoseXInches", Units.metersToInches(pose.getX()));
    SmartDashboard.putNumber("SquareTest/PoseYInches", Units.metersToInches(pose.getY()));
    SmartDashboard.putNumber("SquareTest/PoseHeadingDeg", pose.getRotation().getDegrees());
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static double normalizeRadians(double radians) {
    return Math.atan2(Math.sin(radians), Math.cos(radians));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
