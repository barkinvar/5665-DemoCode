// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.TeleopReefAlignConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.util.MathHelpers;
import java.util.function.Supplier;

public class DriveToPose extends Command {

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          TeleopReefAlignConstants.kPXYController,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              TeleopReefAlignConstants.kXYMaxVelocity, TeleopReefAlignConstants.kXYMaxAccel),
          0.02);

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          TeleopReefAlignConstants.kPThetaController,
          0.0,
          0.2,
          new TrapezoidProfile.Constraints(
              TeleopReefAlignConstants.kThetaMaxVelocity, TeleopReefAlignConstants.kThetaMaxAccel),
          0.02);

  private CommandSwerveDrivetrain driveSubsystem;
  private Supplier<Pose2d> poseSupplier;
  private Translation2d lastSetpointTranslation;
  private double driveErrorAbs;

  private final Timer toleranceTimer = new Timer();
  private final boolean isQuick;

  private ApplyFieldSpeeds mRequest =
      new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity); // TODO

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable driveTargetTable = inst.getTable("DriveToPose");
  private final StructPublisher<Pose2d> alignTargetPose =
      driveTargetTable.getStructTopic("Target", Pose2d.struct).publish();
  private final StructLogEntry<Pose2d> logEntry =
      StructLogEntry.create(DataLogManager.getLog(), "DriveToPose/Target", Pose2d.struct);

  public DriveToPose(Supplier<Pose2d> poseSupplier, boolean quick, CommandSwerveDrivetrain drive) {
    this.poseSupplier = poseSupplier;
    this.isQuick = quick;
    this.driveSubsystem = drive;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    driveController.setTolerance(TeleopReefAlignConstants.kTranslationAllowableError);
    thetaController.setTolerance(TeleopReefAlignConstants.kRotationAllowableError);
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    SwerveDriveState currentState = driveSubsystem.getState();
    Pose2d currentPose = currentState.Pose;
    ChassisSpeeds robotRelativeSpeeds = currentState.Speeds;
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, currentPose.getRotation());

    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(
                    fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
                .rotateBy(
                    poseSupplier
                        .get()
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(currentPose.getRotation().getRadians(), 0.0);
    lastSetpointTranslation = currentPose.getTranslation();
    toleranceTimer.restart();
  }

  @Override
  public void execute() {
    SwerveDriveState currentState = driveSubsystem.getState();
    Pose2d currentPose = currentState.Pose;
    Pose2d targetPose = poseSupplier.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, 0.0)
            + driveController.getSetpoint().velocity * 0.75;
    if (currentDistance
        < TeleopReefAlignConstants.kTranslationAllowableError
            / TeleopReefAlignConstants.kAllowableErrorDivider) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                MathHelpers.transform2dFromTranslation(
                    new Translation2d(driveController.getSetpoint().position, 0.0)))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    double thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs
        < TeleopReefAlignConstants.kRotationAllowableError
            / TeleopReefAlignConstants.kAllowableErrorDivider) thetaVelocity = 0.0;

    if (!driveController.atGoal() || !thetaController.atGoal()) {
      toleranceTimer.reset();
    }

    // Command speeds
    var driveVelocity =
        MathHelpers.pose2dFromRotation(
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(
                MathHelpers.transform2dFromTranslation(new Translation2d(driveVelocityScalar, 0.0)))
            .getTranslation();

    driveSubsystem.setControl(
        mRequest.withSpeeds(
            new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity)));

    Pose2d target =
        new Pose2d(lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position));

    alignTargetPose.set(target);
    logEntry.append(target);
  }

  @Override
  public void end(boolean interrupted) {
    if (!isQuick) driveSubsystem.setControl(mRequest.withSpeeds(new ChassisSpeeds()));
  }

  @Override
  public boolean isFinished() {
    if (isQuick) {
      return toleranceTimer.hasElapsed(AutoAlignConstants.kQuickToleranceTimer);
    }
    return toleranceTimer.hasElapsed(AutoAlignConstants.kToleranceTimer);
  }
}
