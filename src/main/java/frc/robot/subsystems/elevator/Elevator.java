// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstans;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public Elevator() {
    configureMotors();
  }

  TalonFX mMotorLeft = new TalonFX(ElevatorConstans.kLeftMotorID);
  TalonFX mMotorRight = new TalonFX(ElevatorConstans.kRightMotorID);

  private final PositionVoltage mPositionRequest = new PositionVoltage(0.0).withSlot(0);

  private ElevatorTarget setpoint = ElevatorTarget.HOME;

  private void configureMotors() {
    TalonFXConfiguration leftConfig =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(ElevatorConstans.kP).withKD(ElevatorConstans.kD))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(ElevatorConstans.currentLimit))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstans.kRatio));

    mMotorLeft.getConfigurator().apply(leftConfig);

    mMotorRight.setControl(new Follower(0, true));
  }


  public enum ElevatorTarget {
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
    return mMotorLeft.getPosition().getValue().in(Rotations);
  }

  public void resetEncoders() {
    mMotorLeft.setPosition(0.0);
  }

  public Command setForgetSetpointCommand(ElevatorTarget target) {
    return this.runOnce(() -> setSetpoint(target));
  }
  
  public boolean isAtSetPoint(){
    return Math.abs(getHeight() - setpoint.getHeight()) < ElevatorConstans.kTolerance;
  }

  public Command runToSetpoint(ElevatorTarget target) {
    return run(() -> this.setSetpoint(target)).until(this::isAtSetPoint);
  }
  
  @Override
  public void periodic() {
    mMotorLeft.setControl(mPositionRequest.withPosition(setpoint.getHeight()));
  }
}
