// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.kAimPositionToleranceDegrees;
import static frc.robot.Constants.TurretConstants.kClockwiseMotorDirection;
import static frc.robot.Constants.TurretConstants.kMotorRotationsPerTurretRotation;
import static frc.robot.Constants.TurretConstants.kPositionToleranceDegrees;
import static frc.robot.Constants.TurretConstants.kTurretAimVelocityFeedforward;
import static frc.robot.Constants.TurretConstants.kTurretAimMaxOutput;
import static frc.robot.Constants.TurretConstants.kTurretKd;
import static frc.robot.Constants.TurretConstants.kTurretKi;
import static frc.robot.Constants.TurretConstants.kTurretKp;
import static frc.robot.Constants.TurretConstants.kTurretMotorCanId;
import static frc.robot.Constants.TurretConstants.kTurretMovingTargetVelocityThresholdDegreesPerSecond;
import static frc.robot.Constants.TurretConstants.kTurretUnwindMaxOutput;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private final SparkMax m_turretMotor = new SparkMax(kTurretMotorCanId, MotorType.kBrushless);
  private final RelativeEncoder m_turretEncoder = m_turretMotor.getEncoder();

  /** Creates a new TurretSubsystem and treats the startup position as zero. */
  public TurretSubsystem() {
    m_turretEncoder.setPosition(0.0);
  }

  /**
   * Turns the turret to an absolute angle from the startup zero position.
   *
   * @param degrees turret-output degrees from zero
   * @param direction +1 for clockwise, -1 for counterclockwise
   * @param speed motor output from 0.0 to 1.0
   * @return a command that stops when the turret reaches the requested angle
   */
  public Command turnTurretDegrees(double degrees, int direction, double speed) {
    validateTurnRequest(degrees, direction, speed);

    double targetMotorRotations =
        direction * kClockwiseMotorDirection * degreesToMotorRotations(degrees);
    PIDController controller = new PIDController(kTurretKp, kTurretKi, kTurretKd);
    controller.setTolerance(getPositionMotorToleranceRotations());

    return runEnd(
            () -> driveTowardTarget(targetMotorRotations, speed, controller),
            m_turretMotor::stopMotor)
        .beforeStarting(controller::reset)
        .until(() -> atTarget(targetMotorRotations));
  }

  public Command aimAtFixedFieldTargetCommand(Supplier<Pose2d> robotPoseSupplier) {
    return aimAtFixedFieldTargetCommand(robotPoseSupplier, ChassisSpeeds::new);
  }

  public Command aimAtFixedFieldTargetCommand(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    PIDController controller = new PIDController(kTurretKp, kTurretKi, kTurretKd);
    controller.setTolerance(getAimMotorToleranceRotations());

    return runEnd(
            () ->
                aimAtFixedFieldTarget(
                    robotPoseSupplier.get(), chassisSpeedsSupplier.get(), controller),
            m_turretMotor::stopMotor)
        .beforeStarting(controller::reset);
  }

  private void aimAtFixedFieldTarget(
      Pose2d robotPose, ChassisSpeeds chassisSpeeds, PIDController controller) {
    double currentTurretAngleDegrees = motorRotationsToTurretDegrees(m_turretEncoder.getPosition());
    double targetTurretAngleDegrees =
        TurretAiming.calculateTargetAngleDegrees(robotPose, currentTurretAngleDegrees);
    double targetVelocityDegreesPerSecond =
        TurretAiming.calculateTargetAngularVelocityDegreesPerSecond(robotPose, chassisSpeeds);
    double maxOutput =
        TurretAiming.shouldUseUnwindOutput(currentTurretAngleDegrees, targetTurretAngleDegrees)
            ? kTurretUnwindMaxOutput
            : kTurretAimMaxOutput;

    double targetMotorRotations =
        kClockwiseMotorDirection * degreesToMotorRotations(targetTurretAngleDegrees);
    double aimMotorToleranceRotations = getAimMotorToleranceRotations();
    boolean stopAtTolerance =
        Math.abs(targetVelocityDegreesPerSecond)
            <= kTurretMovingTargetVelocityThresholdDegreesPerSecond;
    double feedforwardOutput =
        kClockwiseMotorDirection * targetVelocityDegreesPerSecond * kTurretAimVelocityFeedforward;

    driveTowardTarget(
        targetMotorRotations,
        maxOutput,
        controller,
        stopAtTolerance,
        feedforwardOutput,
        aimMotorToleranceRotations);
    publishAimTelemetry(
        robotPose,
        currentTurretAngleDegrees,
        targetTurretAngleDegrees,
        targetVelocityDegreesPerSecond);
  }

  private void driveTowardTarget(
      double targetMotorRotations, double maxSpeed, PIDController controller) {
    driveTowardTarget(
        targetMotorRotations,
        maxSpeed,
        controller,
        true,
        0.0,
        getPositionMotorToleranceRotations());
  }

  private void driveTowardTarget(
      double targetMotorRotations,
      double maxSpeed,
      PIDController controller,
      boolean stopAtTolerance,
      double feedforwardOutput,
      double toleranceMotorRotations) {
    double currentPosition = m_turretEncoder.getPosition();

    if (stopAtTolerance
        && Math.abs(targetMotorRotations - currentPosition) <= toleranceMotorRotations) {
      m_turretMotor.stopMotor();
      return;
    }

    double pidOutput = controller.calculate(currentPosition, targetMotorRotations);
    m_turretMotor.set(clamp(pidOutput + feedforwardOutput, -maxSpeed, maxSpeed));
  }

  private boolean atTarget(double targetMotorRotations) {
    return Math.abs(targetMotorRotations - m_turretEncoder.getPosition())
        <= getPositionMotorToleranceRotations();
  }

  private static double degreesToMotorRotations(double degrees) {
    return degrees / 360.0 * kMotorRotationsPerTurretRotation;
  }

  private static double motorRotationsToTurretDegrees(double motorRotations) {
    return motorRotations / kClockwiseMotorDirection / kMotorRotationsPerTurretRotation * 360.0;
  }

  private static double getPositionMotorToleranceRotations() {
    return getMotorToleranceRotations(kPositionToleranceDegrees);
  }

  private static double getAimMotorToleranceRotations() {
    return getMotorToleranceRotations(kAimPositionToleranceDegrees);
  }

  private static double getMotorToleranceRotations(double toleranceDegrees) {
    return degreesToMotorRotations(toleranceDegrees);
  }

  private static void publishAimTelemetry(
      Pose2d robotPose,
      double currentTurretAngleDegrees,
      double targetTurretAngleDegrees,
      double targetVelocityDegreesPerSecond) {
    SmartDashboard.putNumber("Turret/CurrentAngleDeg", currentTurretAngleDegrees);
    SmartDashboard.putNumber("Turret/TargetAngleDeg", targetTurretAngleDegrees);
    SmartDashboard.putNumber(
        "Turret/ErrorDeg", targetTurretAngleDegrees - currentTurretAngleDegrees);
    SmartDashboard.putNumber("Turret/TargetVelocityDegPerSec", targetVelocityDegreesPerSecond);
    SmartDashboard.putNumber("Turret/PoseXInches", robotPose.getX() * 39.3700787402);
    SmartDashboard.putNumber("Turret/PoseYInches", robotPose.getY() * 39.3700787402);
    SmartDashboard.putNumber("Turret/PoseHeadingDeg", robotPose.getRotation().getDegrees());
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static void validateTurnRequest(double degrees, int direction, double speed) {
    if (degrees < 0.0) {
      throw new IllegalArgumentException("degrees must be zero or greater");
    }

    if (direction != 1 && direction != -1) {
      throw new IllegalArgumentException("direction must be +1 or -1");
    }

    if (speed <= 0.0 || speed > 1.0) {
      throw new IllegalArgumentException("speed must be greater than 0.0 and no more than 1.0");
    }
  }
}
