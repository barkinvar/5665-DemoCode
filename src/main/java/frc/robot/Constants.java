// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static final String CanivoreName = "Canivore";

  public static final double robotWidthMeters = 0.75;
  public static final double robotLengthMeters = 0.75;
  public static final double bumperWidthMeters = 0.065;


  public static final class AutoAlignConstants {
    public static final double kPXYController = 3.2;
    public static final double kDXYController = 0.008;
    public static final double kPThetaController = 3.0;

    public static final double kXYMaxVelocity = 4.0;
    public static final double kXYMaxAccel = 4.5;
    public static final double kThetaMaxVelocity = Math.toRadians(360.0);
    public static final double kThetaMaxAccel = 8.0;

    public static final double kFFMinRadius = 0.3;
    public static final double kFFMaxRadius = 0.6;

    public static final double kTranslationAllowableError = 0.02;
    public static final double kRotationAllowableError = Math.toRadians(2.0);

    public static final double kAllowableErrorDivider = 2.0;

    public static final double kQuickToleranceTimer = 0.325;
    public static final double kToleranceTimer = 0.125;
  }


  public static final class TeleopReefAlignConstants {
    public static final double kPXYController = 3.75;
    public static final double kPThetaController = 5.5;

    public static final double kXYMaxVelocity = 5.0;
    public static final double kXYMaxAccel = 4.0;
    public static final double kThetaMaxVelocity = Math.toRadians(360.0);
    public static final double kThetaMaxAccel = 8.0;

    public static final double kFFMinRadius = 0.2;
    public static final double kFFMaxRadius = 0.5;

    public static final double kTranslationAllowableError = 0.015;
    public static final double kRotationAllowableError = Math.toRadians(1.2);

    public static final double kAllowableErrorDivider = 1.0;

    public static final double kQuickToleranceTimer = 0.04;
    public static final double kToleranceTimer = 0.12;
  }


  public static final class GondikConstants {
    public static final int kGondikKrakenID = 1;

    public static final Slot0Configs slot0config =
        new Slot0Configs()
            .withKP(20.0)
            .withKI(0.0)
            .withKD(0.5)
            .withKS(0.0)
            .withKG(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
            .withGravityType(GravityTypeValue.Elevator_Static);

    public static final SoftwareLimitSwitchConfigs softLimits =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(0.44)
            .withReverseSoftLimitThreshold(0.0);

    public static final MotorOutputConfigs outputConfig =
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final FeedbackConfigs feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(4.5);

    public static final CurrentLimitsConfigs currentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40.0)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(false);

    public static final double kAllowableError = 0.0;
  }


  public static final class DriveConstants {
    public static final double kSteerJoystickDeadband = 0.02; // TODO
    public static final double kTranslateJoystickDeadband = 0.02;

    public static final double kHeadingControllerP = 2.0;
    public static final double kHeadingControllerI = 0;
    public static final double kHeadingControllerD = 0;

    public static final double kTurnClearanceRatio = 0.008;
  }

  public static class VisionConstants {

    public static final double DISTANCE_WEIGHT = 6;
    public static final int TAG_PRESENCE_WEIGHT = 8;

    public static final double AUTO_DISTANCE_WEIGHT = 15;
    public static final int AUTO_TAG_PRESENCE_WEIGHT = 8;

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
     * then meters.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        VecBuilder.fill(0.34, 0.34, 999999);

    public static final Matrix<N3, N1> AUTO_VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        VecBuilder.fill(0.55, 0.55, 999999);

    public static final Matrix<N3, N1> YAW_VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        VecBuilder.fill(7.0, 7.0, 7.0);

    public static final Matrix<N3, N1> YAW_VISION_MEASUREMENT_CORRECTION_DEVIATIONS =
        VecBuilder.fill(999999, 999999, 10.0);

    public static final Matrix<N3, N1> YAW_VISION_MEASUREMENT_CORRECTION_DEVIATIONS_AUTO =
        VecBuilder.fill(999999, 999999, 4.0);

    public static final double kCameraHeightMeters = 0.78; // Height of the camera in meters
    public static final double kCameraPitchRadians =
        Math.toRadians(30.0); // Pitch angle of the camera
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  public static final class FieldConstants {
    public static final Pose2d[] kHPBlueTargets = {
      new Pose2d(1.4, 8.05 - 7.28, new Rotation2d(Math.toRadians(54.0))),
      new Pose2d(1.4, 7.28, new Rotation2d(Math.toRadians(-54.0)))
    };

    public static final Pose2d[] kHPRedTargets = {
      new Pose2d(17.55 - 1.4, 8.05 - 7.28, new Rotation2d(Math.toRadians(126.0))),
      new Pose2d(17.55 - 1.4, 7.28, new Rotation2d(Math.toRadians(-126.0)))
    };

    private static final double kVerticalOffset = 0.36;
    private static final double kHorizontalOffset = 0.164331;
    private static final Translation2d kLeftTargetOffset =
        new Translation2d(kVerticalOffset, -kHorizontalOffset);
    private static final Translation2d kRightTargetOffset =
        new Translation2d(kVerticalOffset, kHorizontalOffset);
    private static final Translation2d kCenterTargetOffset =
        new Translation2d(kVerticalOffset, 0.0);

    public static final Pose2d[] kReefBlueApriltags = {
      new Pose2d(4.073905999999999, 3.3063179999999996, new Rotation2d(Math.toRadians(240.0))),
      new Pose2d(3.6576, 4.0259, new Rotation2d(Math.toRadians(180.0))),
      new Pose2d(4.073905999999999, 4.745482, new Rotation2d(Math.toRadians(120.0))),
      new Pose2d(4.904739999999999, 4.745482, new Rotation2d(Math.toRadians(60.0))),
      new Pose2d(5.321046, 4.0259, new Rotation2d(Math.toRadians(0.0))),
      new Pose2d(4.904739999999999, 3.3063179999999996, new Rotation2d(Math.toRadians(300.0))),
    };

    public static final Pose2d[] kReefRedApriltags = {
      new Pose2d(13.474446, 3.3063179999999996, new Rotation2d(Math.toRadians(300.0))),
      new Pose2d(13.890498, 4.0259, new Rotation2d(Math.toRadians(0.0))),
      new Pose2d(13.474446, 4.745482, new Rotation2d(Math.toRadians(60.0))),
      new Pose2d(12.643358, 4.745482, new Rotation2d(Math.toRadians(120.0))),
      new Pose2d(12.227305999999999, 4.0259, new Rotation2d(Math.toRadians(180.0))),
      new Pose2d(12.643358, 3.3063179999999996, new Rotation2d(Math.toRadians(240.0)))
    };

    public static final Pose2d[] kReefBlueTargets;
    public static final Pose2d[] kReefRedTargets;

    static {
      int numApriltags = kReefBlueApriltags.length;
      List<Pose2d> blueTargets = new ArrayList<>();
      List<Pose2d> redTargets = new ArrayList<>();

      for (int i = 0; i < numApriltags; i++) {
        blueTargets.add(
            kReefBlueApriltags[i].transformBy(
                new Transform2d(kLeftTargetOffset, new Rotation2d(Math.PI))));
        blueTargets.add(
            kReefBlueApriltags[i].transformBy(
                new Transform2d(kCenterTargetOffset, new Rotation2d(Math.PI))));
        blueTargets.add(
            kReefBlueApriltags[i].transformBy(
                new Transform2d(kRightTargetOffset, new Rotation2d(Math.PI))));
      }

      for (int i = 0; i < numApriltags; i++) {
        redTargets.add(
            kReefRedApriltags[i].transformBy(
                new Transform2d(kLeftTargetOffset, new Rotation2d(Math.PI))));
        redTargets.add(
            kReefRedApriltags[i].transformBy(
                new Transform2d(kCenterTargetOffset, new Rotation2d(Math.PI))));
        redTargets.add(
            kReefRedApriltags[i].transformBy(
                new Transform2d(kRightTargetOffset, new Rotation2d(Math.PI))));
      }

      kReefBlueTargets = blueTargets.toArray(new Pose2d[0]);
      kReefRedTargets = redTargets.toArray(new Pose2d[0]);
    }

    public static final Translation2d kReefBlueCenter =
        new Translation2d(
            (kReefBlueTargets[1].getX() + kReefBlueTargets[10].getX()) / 2.0,
            (kReefBlueTargets[1].getY() + kReefBlueTargets[10].getY()) / 2.0);
    public static final Translation2d kReefRedCenter =
        new Translation2d(
            (kReefRedTargets[1].getX() + kReefRedTargets[10].getX()) / 2.0,
            (kReefRedTargets[1].getY() + kReefRedTargets[10].getY()) / 2.0);
  }

  public static class ElevatorConstans {
    public static final int kLeftMotorID = 15;
    public static final int kRightMotorID = 12;

    public static final double kRatio = 5.0; // TODO

    public static final double kP = 0.0;
    public static final double kD = 0.0;

    public static final double currentLimit = 120.0;

    public static final double kTolerance = 8.0; // TODO
    public static final double pitchDiameter = 152.4 * 2;
  }
}
