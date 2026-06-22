# FRC Turret and Hood Aiming

Java WPILib 2026 robot code for a field-relative turret, distance-adjusted shooter hood, flywheel,
and CTRE swerve drivetrain.

> [!IMPORTANT]
> This project is configured for one specific robot. Do not deploy it unchanged to another robot.
> Verify the CAN IDs, motor directions, gear ratios, encoder zero positions, mechanical limits, PID
> gains, feedforward values, drivetrain geometry, and hood shot map first.

## What It Does

- Continuously aims the turret at a fixed point on the field using the drivetrain's odometry pose.
- Keeps the turret pointed at that field location while the chassis translates or rotates, provided
  odometry is accurate and the target remains within the turret's travel limits.
- Uses chassis velocity feedforward to improve tracking while the robot is moving.
- Selects a safe equivalent turret angle within a `-360 deg` to `+360 deg` cable-wrap range and
  unwinds when necessary.
- Adjusts the hood angle from target distance using a linear map from `30 deg` at `24 in` or closer
  to `45 deg` at `120 in` or farther.
- Runs the flywheel at `1000 RPM` while the driver's right trigger is held.
- Includes a `40 in` square autonomous routine for drivetrain odometry testing.

## Current Target

The turret and hood currently share the fixed field target `(X = 0 in, Y = 120 in)`, configured in
[`Constants.java`](src/main/java/frc/robot/Constants.java). Turret zero is assumed to face field
`+Y`, so a robot starting at `(0, 0, 0 deg)` commands a turret angle of `0 deg`.

The target is field-relative: rotating the chassis changes the required turret angle, and the code
counter-rotates the turret so it continues facing the same field point. Robot translation also
changes the bearing and hood distance continuously.

## Driver Controls

| Control | Action |
| --- | --- |
| Left stick | Field-centric translation |
| Right stick X | Rotation |
| B | Toggle translation inversion |
| Right trigger | Run flywheel at `1000 RPM` while held |
| Right bumper | Toggle hood between `30 deg` and `45 deg` while stationary |

## Hardware Configuration

- REV Spark MAX + NEO 550 turret motor on CAN ID `5`
- CTRE TalonFX flywheel motor on CAN ID `44`
- CTRE TalonFX/Kraken-compatible hood motor on CAN ID `45`
- Turret ratio: `34.6` motor rotations per turret rotation
- Hood ratio: `11.25` motor rotations per hood rotation
- CTRE swerve drivetrain and Pigeon2 configuration in
  [`TunerConstants.java`](src/main/java/frc/robot/generated/TunerConstants.java)

The turret Spark MAX and one drivetrain TalonFX both use numeric CAN ID `5`. This is intentional in
the current robot because they are different device types from different manufacturers.

## Adapting It to Another Robot

1. Update hardware IDs, ratios, motor inversions, limits, and the fixed target in
   [`Constants.java`](src/main/java/frc/robot/Constants.java).
2. Replace the generated swerve configuration with values measured for your chassis.
3. Confirm the turret's physical startup direction before enabling automatic aiming.
4. Put the hood at its physical `30 deg` lower stop before startup, or add a real homing/absolute
   sensor strategy.
5. Retune turret PID and velocity feedforward on the actual mechanism.
6. Characterize the flywheel and replace the placeholder gains and target RPM.
7. Build a measured distance-to-hood-angle shot map for your launcher instead of assuming the
   included linear interpolation will work.
8. Test at low output with the robot disabled between changes and verify all software limits against
   physical hard stops.

## Build and Deploy

Requirements: WPILib 2026 toolchain and Java 17.

```sh
./gradlew build
```

When connected to the roboRIO for team `2584`:

```sh
./gradlew deploy -PteamNumber=2584 --offline
```

## Main Files

- [`TurretSubsystem.java`](src/main/java/frc/robot/subsystems/TurretSubsystem.java)
- [`TurretAiming.java`](src/main/java/frc/robot/subsystems/TurretAiming.java)
- [`HoodSubsystem.java`](src/main/java/frc/robot/subsystems/HoodSubsystem.java)
- [`HoodAiming.java`](src/main/java/frc/robot/subsystems/HoodAiming.java)
- [`FlywheelSubsystem.java`](src/main/java/frc/robot/subsystems/FlywheelSubsystem.java)
- [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java)

## License

This project uses the WPILib BSD license. See [`WPILib-License.md`](WPILib-License.md).
