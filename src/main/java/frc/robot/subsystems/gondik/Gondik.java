// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gondik;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GondikConstants;
import frc.util.CTREUtil;

import java.util.function.Supplier;

public class Gondik extends SubsystemBase {

  private static Gondik mInstance;

  private DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry positionLog = new DoubleLogEntry(log, "Gondik/Position");
  private final DoubleLogEntry targetLog = new DoubleLogEntry(log, "Gondik/Target");
  private final BooleanLogEntry shouldGondikLog = new BooleanLogEntry(log, "Gondik/ShouldGondik");

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable gondikTable = inst.getTable("Gondik");
  private final DoublePublisher positionTopic = gondikTable.getDoubleTopic("Position").publish();
  private final DoublePublisher targetTopic = gondikTable.getDoubleTopic("Target").publish();
  private final BooleanPublisher shouldGondikTopic =
      gondikTable.getBooleanTopic("ShouldGondik").publish();

  public boolean shouldGondik = false;
  private final TalonFX mMotor = new TalonFX(GondikConstants.kGondikKrakenID, "Canivore");
  private final PositionVoltage torqueCurrentControl =
      new PositionVoltage(0.0).withSlot(0);


  public enum GondikTarget {
    HOME(0.25),
    GONDIK_LOW(0.1),
    GONDIK_HIGH(0.25),
    HIGH(0.25);

    private final double degrees;

    private GondikTarget(double height) {
      this.degrees = height;
    }

    public double getDegrees() {
      return degrees;
    }
  }

  private GondikTarget target = GondikTarget.HOME;

  public Gondik() {
    configureMotor();
  }

  private void configureMotor() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withSlot0(GondikConstants.slot0config)
            .withSoftwareLimitSwitch(GondikConstants.softLimits)
            .withMotorOutput(GondikConstants.outputConfig)
            .withFeedback(GondikConstants.feedbackConfig)
            .withCurrentLimits(GondikConstants.currentLimits);
    CTREUtil.applyConfiguration(mMotor, config);
  }

  @Override
  public void periodic() {
    mMotor.setControl(torqueCurrentControl.withPosition(target.getDegrees()));

    final double position = getPosition();
    positionLog.append(position);
    positionTopic.set(position);

    targetLog.append(target.getDegrees());
    targetTopic.set(target.getDegrees());

    shouldGondikLog.append(shouldGondik);
    shouldGondikTopic.set(shouldGondik);
  }

  public double getPosition() {
    return mMotor.getPosition().getValue().in(Degrees);
  }

  public void resetEncoder() {
    CTREUtil.tryUntilOK(() -> mMotor.setPosition(0.27), 2);
  }

  public void setTarget(GondikTarget target) {
    this.target = target;
  }

  public Command setTargetCommand(Supplier<GondikTarget> target) {
    return run(() -> this.target = target.get()).withTimeout(0.25);
  }

  public Command setTargetCommand(GondikTarget target) {
    return run(() -> this.target = target).withTimeout(0.25);
  }
}
