package frc.robot.subsystems;

import static frc.robot.Constants.TurretConstants.kFixedTargetXInches;
import static frc.robot.Constants.TurretConstants.kFixedTargetYInches;
import static frc.robot.Constants.TurretConstants.kTurretMaxAngleDegrees;
import static frc.robot.Constants.TurretConstants.kTurretMinAngleDegrees;
import static frc.robot.Constants.TurretConstants.kTurretZeroFieldHeadingDegrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

final class TurretAiming {
  private static final double kFullTurnDegrees = 360.0;
  private static final double kHalfTurnDegrees = 180.0;
  private static final double kTargetDeadbandMeters = 1e-9;

  static double calculateTargetAngleDegrees(
      Pose2d robotPose, double currentTurretAngleDegrees) {
    double targetX = Units.inchesToMeters(kFixedTargetXInches);
    double targetY = Units.inchesToMeters(kFixedTargetYInches);
    double deltaX = targetX - robotPose.getX();
    double deltaY = targetY - robotPose.getY();

    if (Math.hypot(deltaX, deltaY) < kTargetDeadbandMeters) {
      return currentTurretAngleDegrees;
    }

    double targetBearingDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));
    double desiredAngleDegrees =
        robotPose.getRotation().getDegrees()
            + kTurretZeroFieldHeadingDegrees
            - targetBearingDegrees;

    return chooseClosestSafeEquivalentDegrees(desiredAngleDegrees, currentTurretAngleDegrees);
  }

  static double calculateTargetAngularVelocityDegreesPerSecond(
      Pose2d robotPose, ChassisSpeeds robotRelativeChassisSpeeds) {
    double targetX = Units.inchesToMeters(kFixedTargetXInches);
    double targetY = Units.inchesToMeters(kFixedTargetYInches);
    double deltaX = targetX - robotPose.getX();
    double deltaY = targetY - robotPose.getY();
    double distanceSquared = deltaX * deltaX + deltaY * deltaY;

    if (distanceSquared < kTargetDeadbandMeters * kTargetDeadbandMeters) {
      return 0.0;
    }

    double headingCos = robotPose.getRotation().getCos();
    double headingSin = robotPose.getRotation().getSin();
    double fieldVelocityX =
        robotRelativeChassisSpeeds.vxMetersPerSecond * headingCos
            - robotRelativeChassisSpeeds.vyMetersPerSecond * headingSin;
    double fieldVelocityY =
        robotRelativeChassisSpeeds.vxMetersPerSecond * headingSin
            + robotRelativeChassisSpeeds.vyMetersPerSecond * headingCos;
    double targetBearingVelocityRadiansPerSecond =
        (deltaY * fieldVelocityX - deltaX * fieldVelocityY) / distanceSquared;

    return Math.toDegrees(
        robotRelativeChassisSpeeds.omegaRadiansPerSecond - targetBearingVelocityRadiansPerSecond);
  }

  static double chooseClosestSafeEquivalentDegrees(
      double desiredAngleDegrees, double currentTurretAngleDegrees) {
    double baseAngleDegrees = normalizeToHalfTurn(desiredAngleDegrees);
    double bestAngleDegrees = baseAngleDegrees;
    double bestDistanceDegrees = Double.POSITIVE_INFINITY;

    for (int turnOffset = -1; turnOffset <= 1; turnOffset++) {
      double candidateDegrees = baseAngleDegrees + turnOffset * kFullTurnDegrees;
      if (candidateDegrees < kTurretMinAngleDegrees
          || candidateDegrees > kTurretMaxAngleDegrees) {
        continue;
      }

      double distanceDegrees = Math.abs(candidateDegrees - currentTurretAngleDegrees);
      if (distanceDegrees < bestDistanceDegrees) {
        bestAngleDegrees = candidateDegrees;
        bestDistanceDegrees = distanceDegrees;
      }
    }

    return bestAngleDegrees;
  }

  static boolean shouldUseUnwindOutput(
      double currentTurretAngleDegrees, double targetTurretAngleDegrees) {
    return Math.abs(targetTurretAngleDegrees - currentTurretAngleDegrees) > kHalfTurnDegrees;
  }

  private static double normalizeToHalfTurn(double degrees) {
    double normalizedDegrees = degrees % kFullTurnDegrees;

    if (normalizedDegrees > kHalfTurnDegrees) {
      normalizedDegrees -= kFullTurnDegrees;
    } else if (normalizedDegrees <= -kHalfTurnDegrees) {
      normalizedDegrees += kFullTurnDegrees;
    }

    return normalizedDegrees;
  }

  private TurretAiming() {}
}
