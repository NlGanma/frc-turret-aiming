package frc.robot.subsystems;

import static frc.robot.Constants.HoodConstants.kHoodFarDistanceInches;
import static frc.robot.Constants.HoodConstants.kHoodMaxAngleDegrees;
import static frc.robot.Constants.HoodConstants.kHoodMinAngleDegrees;
import static frc.robot.Constants.HoodConstants.kHoodNearDistanceInches;
import static frc.robot.Constants.TurretConstants.kFixedTargetXInches;
import static frc.robot.Constants.TurretConstants.kFixedTargetYInches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

final class HoodAiming {
  static double calculateTargetAngleDegrees(Pose2d robotPose) {
    return calculateTargetAngleDegrees(calculateDistanceInches(robotPose));
  }

  static double calculateTargetAngleDegrees(double distanceInches) {
    double interpolation =
        (distanceInches - kHoodNearDistanceInches)
            / (kHoodFarDistanceInches - kHoodNearDistanceInches);

    return clamp(
        kHoodMinAngleDegrees
            + interpolation * (kHoodMaxAngleDegrees - kHoodMinAngleDegrees),
        kHoodMinAngleDegrees,
        kHoodMaxAngleDegrees);
  }

  static double calculateDistanceInches(Pose2d robotPose) {
    double targetXMeters = Units.inchesToMeters(kFixedTargetXInches);
    double targetYMeters = Units.inchesToMeters(kFixedTargetYInches);
    double distanceMeters =
        Math.hypot(targetXMeters - robotPose.getX(), targetYMeters - robotPose.getY());

    return Units.metersToInches(distanceMeters);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private HoodAiming() {}
}
