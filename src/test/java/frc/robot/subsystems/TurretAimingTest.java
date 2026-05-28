package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class TurretAimingTest {
  private static final double kTolerance = 1e-9;

  @Test
  void startupPoseAimsAtZeroDegrees() {
    Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);

    assertEquals(0.0, TurretAiming.calculateTargetAngleDegrees(robotPose, 0.0), kTolerance);
  }

  @Test
  void chassisRotationCommandsClockwiseTurretAngle() {
    Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0));

    assertEquals(90.0, TurretAiming.calculateTargetAngleDegrees(robotPose, 0.0), kTolerance);
  }

  @Test
  void translatedPoseUsesBearingToFixedTarget() {
    Pose2d robotPose = new Pose2d(Units.inchesToMeters(120.0), 0.0, Rotation2d.kZero);

    assertEquals(-45.0, TurretAiming.calculateTargetAngleDegrees(robotPose, 0.0), kTolerance);
  }

  @Test
  void unsafeNearestEquivalentUnwindsInsideTravelRange() {
    assertEquals(
        20.0, TurretAiming.chooseClosestSafeEquivalentDegrees(20.0, 350.0), kTolerance);
    assertTrue(TurretAiming.shouldUseUnwindOutput(350.0, 20.0));
  }

  @Test
  void safeNearestEquivalentDoesNotUnwind() {
    assertEquals(
        340.0, TurretAiming.chooseClosestSafeEquivalentDegrees(-20.0, 350.0), kTolerance);
    assertFalse(TurretAiming.shouldUseUnwindOutput(350.0, 340.0));
  }

  @Test
  void chassisRotationAddsTargetAngularVelocity() {
    Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, Math.toRadians(90.0));

    assertEquals(
        90.0,
        TurretAiming.calculateTargetAngularVelocityDegreesPerSecond(robotPose, chassisSpeeds),
        kTolerance);
  }

  @Test
  void fieldTranslationContributesBearingAngularVelocity() {
    Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);
    double targetDistanceMeters = Units.inchesToMeters(120.0);
    double expectedDegreesPerSecond = -Math.toDegrees(1.0 / targetDistanceMeters);

    assertEquals(
        expectedDegreesPerSecond,
        TurretAiming.calculateTargetAngularVelocityDegreesPerSecond(robotPose, chassisSpeeds),
        kTolerance);
  }
}
