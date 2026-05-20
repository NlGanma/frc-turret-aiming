// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final double m_maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.35;
  private final double m_maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(m_maxSpeed * 0.1)
          .withRotationalDeadband(m_maxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private double m_driveDirectionMultiplier = 1.0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(
            () ->
                m_drive
                    .withVelocityX(
                        -m_driverController.getLeftY() * m_maxSpeed * m_driveDirectionMultiplier)
                    .withVelocityY(
                        -m_driverController.getLeftX() * m_maxSpeed * m_driveDirectionMultiplier)
                    .withRotationalRate(-m_driverController.getRightX() * m_maxAngularRate)));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_driveDirectionMultiplier *= -1.0;
                  SmartDashboard.putBoolean("Drive Inverted", m_driveDirectionMultiplier < 0);
                }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.turnTurretNinetyDegreesAndBack(m_turretSubsystem);
  }
}
