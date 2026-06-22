# AGENTS.md

Guidance for AI agents and teammates working in this project. Update this file whenever robot
hardware mappings, autonomous behavior, subsystem APIs, tuning constants, or build/deploy workflow
change.

## Project Overview

- This is a Java WPILib 2026 command-based robot project.
- Team number is `2584` in `.wpilib/wpilib_preferences.json`.
- Build with `./gradlew build`.
- Deploy with `./gradlew deploy -PteamNumber=2584 --offline` when connected to the roboRIO.
- REVLib is installed through `vendordeps/REVLib.json`.
- CTRE Phoenix 6 is installed through `vendordeps/Phoenix6-26.1.0.json`.

## Current Robot Behavior

- `RobotContainer.getAutonomousCommand()` runs the drivetrain odometry square test.
- The current autonomous sequence is `Autos.driveFortyInchSquare(...)`.
- Autonomous resets drivetrain pose to `(0, 0, 0°)`, drives a `40 in` by `40 in` field-centric
  square through `(40, 0)`, `(40, 40)`, `(0, 40)`, and `(0, 0)`, then idles the drivetrain.
- The square test publishes `SquareTest/Waypoint`, target X/Y inches, pose X/Y inches, and pose
  heading to SmartDashboard.
- Current square test speed cap is `24 in/s`.
- Teleop default drive runs the CTRE swerve drivetrain field-centrically.
- Driver controls match frankenbot: left stick translates, right stick X rotates, and B toggles
  translation inversion.
- Driver RT runs the flywheel at `1000 RPM` while held; releasing RT disables output and lets the
  flywheel coast down.
- The hood automatically tracks the same fixed field target as the turret, mapping target distance
  to `30°` through `45°`.

## Swerve Chassis

- Main files:
  - `src/main/java/frc/robot/generated/TunerConstants.java`
  - `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`
- The chassis configuration mirrors `2026-Robot-Code-frankenbot`.
- `RobotContainer` seeds drivetrain pose to `(0, 0, 0°)` at program startup.
- Pigeon2 CAN ID: `1`
- Front left: drive `1`, steer `2`, CANcoder `1`
- Front right: drive `3`, steer `4`, CANcoder `2`
- Back left: drive `5`, steer `6`, CANcoder `3`
- Back right: drive `7`, steer `8`, CANcoder `4`
- Swerve odometry is available through `CommandSwerveDrivetrain.getDrivePose2d()`.
- The REV turret Spark MAX and CTRE back-left drive TalonFX both use numeric CAN ID `5`; this is
  intentional because they are different device types/manufacturers.

## Turret Subsystem

- Main file: `src/main/java/frc/robot/subsystems/TurretSubsystem.java`.
- Public command method:

```java
Command turnTurretDegrees(double degrees, int direction, double speed)
```

- Default command:

```java
Command aimAtFixedFieldTargetCommand(Supplier<Pose2d> robotPoseSupplier)
Command aimAtFixedFieldTargetCommand(
    Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier)
```

- `degrees` means final turret-output degrees from the program-start zero position.
- `direction` must be `+1` for clockwise or `-1` for counterclockwise.
- `speed` must be greater than `0.0` and no more than `1.0`; it caps PID motor output.
- The Spark MAX built-in relative encoder is zeroed once in the subsystem constructor.
- Do not reset the encoder inside each move command unless the homing strategy changes.
- The command stops the motor when it reaches tolerance or is interrupted.
- The default turret command aims at the fixed field point `(0, 120 in)` using drivetrain pose.
- The default turret command also uses drivetrain chassis speeds for moving-target velocity
  feedforward and to avoid stopping inside tolerance while the target angle is still moving.
- The default turret command publishes `Turret/PoseXInches`, `Turret/PoseYInches`,
  `Turret/PoseHeadingDeg`, `Turret/TargetAngleDeg`, and `Turret/ErrorDeg` to SmartDashboard.
- Turret zero is assumed to face field `+Y`, so startup pose `(0, 0, 0°)` commands `0°`.
- The cable-wrap travel range is `-360°` to `+360°`; when the nearest equivalent angle is outside
  that range, the turret chooses the closest safe equivalent and unwinds quickly.

## Turret Hardware and Tuning

Constants live in `src/main/java/frc/robot/Constants.java`.

- Motor controller: REV Spark MAX
- CAN ID: `5`
- Motor: NEO 550 brushless
- Encoder: Spark MAX built-in relative encoder
- Gear ratio: `34.6` motor rotations per `1` turret rotation
- Clockwise motor sign: `kClockwiseMotorDirection = 1.0`
- Position tolerance: `1.0` turret degree
- Continuous aim position tolerance: `0.25` turret degree
- PID values:
  - `kTurretKp = 0.1285`
  - `kTurretKi = 0.0`
  - `kTurretKd = 0.0024`
- Continuous aim velocity feedforward:
  - `kTurretAimVelocityFeedforward = 0.0005`
  - `kTurretMovingTargetVelocityThresholdDegreesPerSecond = 5.0`

If the turret moves counterclockwise when commanded clockwise, flip `kClockwiseMotorDirection`
instead of changing call sites.

## Flywheel Subsystem

- Main file: `src/main/java/frc/robot/subsystems/FlywheelSubsystem.java`.
- Motor controller: CTRE TalonFX
- CAN ID: `44`
- Driver control: Xbox right trigger on port `0`, active above `0.5`
- Target velocity: `1000 RPM`
- The subsystem uses Phoenix 6 `VelocityVoltage` closed-loop control with the TalonFX integrated
  sensor.
- Releasing RT calls `stopMotor()` and uses coast neutral mode; the flywheel is not actively braked
  to zero.
- Direction is controlled by `kFlywheelInverted`; flip that constant if positive `1000 RPM` spins
  the wrong direction.
- Flywheel tuning constants:
  - `kFlywheelKp = 0.0`
  - `kFlywheelKi = 0.0`
  - `kFlywheelKd = 0.0`
  - `kFlywheelKs = 0.0`
  - `kFlywheelKv = 0.12`
  - `kFlywheelKa = 0.0`
- SmartDashboard publishes `Flywheel/Active`, `Flywheel/TargetRpm`, and
  `Flywheel/MeasuredRpm`.

## Hood Subsystem

- Main file: `src/main/java/frc/robot/subsystems/HoodSubsystem.java`.
- Motor controller: CTRE TalonFX/Kraken-compatible Phoenix 6 device
- CAN ID: `45`
- Gear ratio: `225:20`, or `11.25` motor rotations per `1` hood rotation
- The hood travel range is `30°` to `45°`; `30°` is the physical lower stop.
- The hood uses Phoenix 6 `PositionVoltage` closed-loop control with FOC disabled.
- The TalonFX integrated sensor is configured with `SensorToMechanismRatio = 11.25`, so position
  requests and soft limits are in hood mechanism rotations.
- On startup, the hood encoder is seeded to `30°`; make sure the hood is physically at the lower
  stop before booting unless a homing or absolute sensor is added.
- The hood default command aims continuously at the same fixed field point as the turret,
  `(0, 120 in)`, using drivetrain pose.
- Driver RB toggles a manual hood override between `30°` and `45°`, but only when drivetrain
  chassis speeds are below `0.05 m/s` translation and `0.05 rad/s` rotation. While the manual
  override is active, the hood holds the toggled min/max angle instead of distance aiming.
- Distance-to-angle map:
  - `24 in` or closer commands `30°`
  - `120 in` or farther commands `45°`
  - distances between those values linearly interpolate between `30°` and `45°`
- Direction is controlled by `kHoodInverted`; flip that constant if the hood moves the wrong way.
- Initial hood tuning constants:
  - `kHoodKp = 24.0`
  - `kHoodKi = 0.0`
  - `kHoodKd = 0.0`
  - `kHoodKs = 0.0`
  - `kHoodKv = 0.0`
  - `kHoodKa = 0.0`
  - `kHoodMaxVoltage = 4.0`
- SmartDashboard publishes `Hood/DistanceInches`, `Hood/TargetAngleDeg`,
  `Hood/CurrentAngleDeg`, `Hood/ErrorDeg`, `Hood/ManualOverride`, and
  `Hood/RBToggleAccepted`.

## Code Conventions

- Keep robot-wide constants in `Constants.java`.
- Put hardware ownership in subsystems, not commands or `Robot.java`.
- Prefer subsystem command factory methods for robot actions.
- For any command that moves a motor, make sure the motor stops on finish and interruption.
- Keep changes narrow; do not remove default example code unless it directly blocks the requested
  robot behavior.

## Update Checklist

Update this file when adding or changing:

- CAN IDs, motor types, gear ratios, sensor sources, or wiring assumptions.
- Public subsystem methods or command signatures.
- Autonomous routines or default command behavior.
- PID/feedforward constants, tolerances, speed caps, or direction conventions.
- Build, deploy, simulation, or Driver Station workflow.
