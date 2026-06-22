package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class HoodAimingTest {
  private static final double kTolerance = 1e-9;

  @Test
  void closeDistanceClampsToMinimumAngle() {
    assertEquals(30.0, HoodAiming.calculateTargetAngleDegrees(12.0), kTolerance);
  }

  @Test
  void farDistanceClampsToMaximumAngle() {
    assertEquals(45.0, HoodAiming.calculateTargetAngleDegrees(180.0), kTolerance);
  }

  @Test
  void midpointDistanceInterpolatesBetweenMinimumAndMaximumAngles() {
    assertEquals(37.5, HoodAiming.calculateTargetAngleDegrees(72.0), kTolerance);
  }

  @Test
  void robotAtFixedTargetUsesMinimumAngle() {
    Pose2d robotPose =
        new Pose2d(0.0, Units.inchesToMeters(120.0), Rotation2d.kZero);

    assertEquals(30.0, HoodAiming.calculateTargetAngleDegrees(robotPose), kTolerance);
  }
}
