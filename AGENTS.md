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

- `RobotContainer.getAutonomousCommand()` runs the turret autonomous sequence.
- The current autonomous sequence is `Autos.turnTurretNinetyDegreesAndBack(...)`.
- Autonomous turns the turret gear `90` degrees clockwise from startup zero, then returns to `0`.
- Current autonomous speed cap is `0.6`.
- Teleop default drive runs the CTRE swerve drivetrain field-centrically.
- Driver controls match frankenbot: left stick translates, right stick X rotates, and B toggles
  translation inversion.

## Swerve Chassis

- Main files:
  - `src/main/java/frc/robot/generated/TunerConstants.java`
  - `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`
- The chassis configuration mirrors `2026-Robot-Code-frankenbot`.
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

- `degrees` means final turret-output degrees from the program-start zero position.
- `direction` must be `+1` for clockwise or `-1` for counterclockwise.
- `speed` must be greater than `0.0` and no more than `1.0`; it caps PID motor output.
- The Spark MAX built-in relative encoder is zeroed once in the subsystem constructor.
- Do not reset the encoder inside each move command unless the homing strategy changes.
- The command stops the motor when it reaches tolerance or is interrupted.

## Turret Hardware and Tuning

Constants live in `src/main/java/frc/robot/Constants.java`.

- Motor controller: REV Spark MAX
- CAN ID: `5`
- Motor: NEO 550 brushless
- Encoder: Spark MAX built-in relative encoder
- Gear ratio: `34.6` motor rotations per `1` turret rotation
- Clockwise motor sign: `kClockwiseMotorDirection = 1.0`
- Position tolerance: `1.0` turret degree
- PID values:
  - `kTurretKp = 0.1285`
  - `kTurretKi = 0.0`
  - `kTurretKd = 0.0024`

If the turret moves counterclockwise when commanded clockwise, flip `kClockwiseMotorDirection`
instead of changing call sites.

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
