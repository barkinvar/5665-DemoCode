// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelSubsystem extends SubsystemBase {
  private final TalonFX mMotor = new TalonFX(2, "Canivore");

  private final CANrange mSensor = new CANrange(14, "Canivore");
  private final VoltageOut mRequest = new VoltageOut(0.0);

  private final Debouncer mDebouncer = new Debouncer(0.04);

  /** Creates a new FunnelSubsystem. */
  public FunnelSubsystem() {
    configMotor();
  }

  private void configMotor() {
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(100.0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(false));

    mMotor.getConfigurator().apply(motorConfig);

    CANrangeConfiguration sensorConfig =
        new CANrangeConfiguration()
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withMinSignalStrengthForValidMeasurement(1500.0)
                    .withProximityThreshold(0.2))
            .withToFParams(
                new ToFParamsConfigs()
                    .withUpdateFrequency(100)
                    .withUpdateMode(UpdateModeValue.ShortRange100Hz))
            .withFovParams(new FovParamsConfigs().withFOVRangeX(10.0).withFOVRangeY(10.0));

    mSensor.getConfigurator().apply(sensorConfig);
  }

  public void setVoltage(double voltage) {
    mMotor.setControl(mRequest.withOutput(voltage));
  }

  public boolean getSensorState() {
    return mSensor.getIsDetected().getValue();
  }

  public Command runHopperAtVoltageWithSensor(double voltage) {
    return runOnce(() -> mDebouncer.calculate(false))
        .andThen(
            runEnd(() -> setVoltage(voltage), () -> setVoltage(0.0))
                .until(() -> mDebouncer.calculate(getSensorState())))
        .unless(this::getSensorState);
  }

  public Command shoot(double voltage){
    return runEnd(() -> setVoltage(voltage), () -> setVoltage(0.0)).until(() -> !getSensorState()).unless(() -> !getSensorState());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
