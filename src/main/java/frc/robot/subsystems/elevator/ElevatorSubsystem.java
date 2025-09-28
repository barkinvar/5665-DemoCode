// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstans;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
  public ElevatorSubsystem() {
    configureMotors();
  }

  TalonFX mMotorLeft = new TalonFX(ElevatorConstans.kLeftMotorID, "Canivore");
  TalonFX mMotorRight = new TalonFX(ElevatorConstans.kRightMotorID, "Canivore");

  private final PositionVoltage mPositionRequest = new PositionVoltage(0.0).withSlot(0);
  private final CANrange mSensor = new CANrange(14, "Canivore");

  private ElevatorTarget setpoint = ElevatorTarget.STOP;

  private void configureMotors() {
    TalonFXConfiguration leftConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(ElevatorConstans.kP).withKD(ElevatorConstans.kD).withKS(0.3).withKG(0.8).withKP(12.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ElevatorConstans.currentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(false))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(500.0 / ElevatorConstans.pitchDiameter)
                    .withReverseSoftLimitThreshold(0.0))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstans.kRatio));

    mMotorLeft.getConfigurator().apply(leftConfig);
    mMotorRight.getConfigurator().apply(leftConfig);

    mMotorRight.setControl(new Follower(ElevatorConstans.kLeftMotorID, true));

    CANrangeConfiguration sensorConfig =
        new CANrangeConfiguration()
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withMinSignalStrengthForValidMeasurement(1500.0)
                    .withProximityThreshold(0.2))
            .withToFParams(
                new ToFParamsConfigs()
                    .withUpdateFrequency(100)
                    .withUpdateMode(UpdateModeValue.ShortRange100Hz));

    mSensor.getConfigurator().apply(sensorConfig);
  }

  public enum ElevatorTarget {
    STOP(0.0),
    HOME(0.0),
    L1(300.0),
    L2(500.0),
    L3(900.0),
    L4(1500.0);

    private final double height;

    private ElevatorTarget(double height) {
      this.height = height;
    }

    public double getHeight() {
      return height;
    }
  }

  public void setSetpoint(ElevatorTarget target) {
    setpoint = target;
  }

  public ElevatorTarget getSetpoint() {
    return setpoint;
  }

  public double getHeight() {
    return mMotorLeft.getPosition().getValue().in(Rotations) * ElevatorConstans.pitchDiameter;
  }

  public void resetEncoders() {
    mMotorLeft.setPosition(0.0);
    mMotorRight.setPosition(0.0);
  }

  public Command setForgetSetpointCommand(ElevatorTarget target) {
    return this.runOnce(() -> setSetpoint(target));
  }

  public boolean isAtSetPoint() {
    return Math.abs(getHeight() - setpoint.getHeight()) < ElevatorConstans.kTolerance;
  }

  public Command runToSetpoint(ElevatorTarget target) {
    return run(() -> this.setSetpoint(target)).until(this::isAtSetPoint);
  }

  @Override
  public void periodic() {
    switch (setpoint) {
      case STOP:
        mMotorLeft.set(0.0);
        break;
      default:
        mMotorLeft.setControl(mPositionRequest.withPosition(setpoint.getHeight() / ElevatorConstans.pitchDiameter));
        break;
    }
    SmartDashboard.putNumber("Elevator Height", getHeight());

  }
}
