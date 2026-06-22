// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.FlywheelConstants.kFlywheelInverted;
import static frc.robot.Constants.FlywheelConstants.kFlywheelKa;
import static frc.robot.Constants.FlywheelConstants.kFlywheelKd;
import static frc.robot.Constants.FlywheelConstants.kFlywheelKi;
import static frc.robot.Constants.FlywheelConstants.kFlywheelKp;
import static frc.robot.Constants.FlywheelConstants.kFlywheelKs;
import static frc.robot.Constants.FlywheelConstants.kFlywheelKv;
import static frc.robot.Constants.FlywheelConstants.kFlywheelMotorCanId;
import static frc.robot.Constants.FlywheelConstants.kFlywheelTargetRpm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
  private static final double kSecondsPerMinute = 60.0;

  private final TalonFX m_flywheelMotor = new TalonFX(kFlywheelMotorCanId);
  private final VelocityVoltage m_velocityRequest =
      new VelocityVoltage(rpmToRotationsPerSecond(kFlywheelTargetRpm)).withEnableFOC(false);

  private boolean m_isRunning = false;

  /** Creates a flywheel subsystem using the TalonFX integrated velocity sensor. */
  public FlywheelSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    configs.MotorOutput.Inverted =
        kFlywheelInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    configs.Slot0.kP = kFlywheelKp;
    configs.Slot0.kI = kFlywheelKi;
    configs.Slot0.kD = kFlywheelKd;
    configs.Slot0.kS = kFlywheelKs;
    configs.Slot0.kV = kFlywheelKv;
    configs.Slot0.kA = kFlywheelKa;

    m_flywheelMotor.getConfigurator().apply(configs);
  }

  /**
   * Runs the flywheel at the configured target RPM until the command ends or is interrupted.
   *
   * @return a command that coasts the flywheel when it ends
   */
  public Command runFlywheelCommand() {
    return runEnd(this::runAtTargetRpm, this::stop);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Flywheel/Active", m_isRunning);
    SmartDashboard.putNumber("Flywheel/TargetRpm", m_isRunning ? kFlywheelTargetRpm : 0.0);
    SmartDashboard.putNumber("Flywheel/MeasuredRpm", getMeasuredRpm());
  }

  private void runAtTargetRpm() {
    m_isRunning = true;
    m_flywheelMotor.setControl(m_velocityRequest);
  }

  private void stop() {
    m_isRunning = false;
    m_flywheelMotor.stopMotor();
  }

  private double getMeasuredRpm() {
    return m_flywheelMotor.getVelocity().getValue().in(RotationsPerSecond) * kSecondsPerMinute;
  }

  private static double rpmToRotationsPerSecond(double rpm) {
    return rpm / kSecondsPerMinute;
  }
}
