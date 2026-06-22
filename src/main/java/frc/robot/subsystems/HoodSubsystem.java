// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.HoodConstants.kHoodInverted;
import static frc.robot.Constants.HoodConstants.kHoodKa;
import static frc.robot.Constants.HoodConstants.kHoodKd;
import static frc.robot.Constants.HoodConstants.kHoodKi;
import static frc.robot.Constants.HoodConstants.kHoodKp;
import static frc.robot.Constants.HoodConstants.kHoodKs;
import static frc.robot.Constants.HoodConstants.kHoodKv;
import static frc.robot.Constants.HoodConstants.kHoodMaxAngleDegrees;
import static frc.robot.Constants.HoodConstants.kHoodMaxVoltage;
import static frc.robot.Constants.HoodConstants.kHoodMinAngleDegrees;
import static frc.robot.Constants.HoodConstants.kHoodMotorCanId;
import static frc.robot.Constants.HoodConstants.kMotorRotationsPerHoodRotation;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
  private static final double kDegreesPerRotation = 360.0;

  private final TalonFX m_hoodMotor = new TalonFX(kHoodMotorCanId);
  private final PositionVoltage m_positionRequest =
      new PositionVoltage(degreesToMechanismRotations(kHoodMinAngleDegrees))
          .withEnableFOC(false);

  private double m_targetAngleDegrees = kHoodMinAngleDegrees;
  private double m_targetDistanceInches = 0.0;
  private boolean m_manualAngleOverrideEnabled = false;
  private boolean m_manualAngleOverrideAtMax = false;

  /** Creates a hood subsystem and assumes the hood starts at the 30 degree lower stop. */
  public HoodSubsystem() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted =
        kHoodInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    configs.Feedback.SensorToMechanismRatio = kMotorRotationsPerHoodRotation;
    configs.Voltage.PeakForwardVoltage = kHoodMaxVoltage;
    configs.Voltage.PeakReverseVoltage = -kHoodMaxVoltage;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        degreesToMechanismRotations(kHoodMaxAngleDegrees);
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        degreesToMechanismRotations(kHoodMinAngleDegrees);
    configs.Slot0.kP = kHoodKp;
    configs.Slot0.kI = kHoodKi;
    configs.Slot0.kD = kHoodKd;
    configs.Slot0.kS = kHoodKs;
    configs.Slot0.kV = kHoodKv;
    configs.Slot0.kA = kHoodKa;

    m_hoodMotor.getConfigurator().apply(configs);
    m_hoodMotor.setPosition(degreesToMechanismRotations(kHoodMinAngleDegrees));
  }

  public Command aimAtFixedFieldTargetCommand(Supplier<Pose2d> robotPoseSupplier) {
    return runEnd(() -> aimAtFixedFieldTarget(robotPoseSupplier.get()), m_hoodMotor::stopMotor);
  }

  public void toggleMinMaxAngleOverride() {
    m_manualAngleOverrideEnabled = true;
    m_manualAngleOverrideAtMax = !m_manualAngleOverrideAtMax;
    m_targetAngleDegrees =
        m_manualAngleOverrideAtMax ? kHoodMaxAngleDegrees : kHoodMinAngleDegrees;
  }

  @Override
  public void periodic() {
    double currentAngleDegrees = getCurrentAngleDegrees();
    SmartDashboard.putBoolean("Hood/ManualOverride", m_manualAngleOverrideEnabled);
    SmartDashboard.putNumber("Hood/DistanceInches", m_targetDistanceInches);
    SmartDashboard.putNumber("Hood/TargetAngleDeg", m_targetAngleDegrees);
    SmartDashboard.putNumber("Hood/CurrentAngleDeg", currentAngleDegrees);
    SmartDashboard.putNumber("Hood/ErrorDeg", m_targetAngleDegrees - currentAngleDegrees);
  }

  private void aimAtFixedFieldTarget(Pose2d robotPose) {
    m_targetDistanceInches = HoodAiming.calculateDistanceInches(robotPose);
    if (!m_manualAngleOverrideEnabled) {
      m_targetAngleDegrees = HoodAiming.calculateTargetAngleDegrees(m_targetDistanceInches);
    }
    commandAngleDegrees(m_targetAngleDegrees);
  }

  private void commandAngleDegrees(double angleDegrees) {
    m_hoodMotor.setControl(
        m_positionRequest.withPosition(degreesToMechanismRotations(angleDegrees)));
  }

  private double getCurrentAngleDegrees() {
    return mechanismRotationsToDegrees(m_hoodMotor.getPosition().getValue().in(Rotations));
  }

  private static double degreesToMechanismRotations(double degrees) {
    return degrees / kDegreesPerRotation;
  }

  private static double mechanismRotationsToDegrees(double rotations) {
    return rotations * kDegreesPerRotation;
  }
}
