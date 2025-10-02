package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.util.LimelightHelpers.Limelight;
import frc.util.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
  private DataLog log = DataLogManager.getLog();

  private final Limelight leftLimelight = new Limelight("limelight-left");
  private final Limelight rightLimelight = new Limelight("limelight-right");
  private final Limelight coralLimelight = new Limelight("limelight-coral");

  private final StructLogEntry<Pose2d> leftLimelightLog =
      StructLogEntry.create(log, "Vision/LeftLimelight", Pose2d.struct);
  private final StructLogEntry<Pose2d> rightLimelightLog =
      StructLogEntry.create(log, "Vision/RightLimelight", Pose2d.struct);

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable visionTable = inst.getTable("Vision");
  private final StructPublisher<Pose2d> leftLimelightPoseTable =
      visionTable.getStructTopic("LeftLimelight", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> rightLimelightPoseTable =
      visionTable.getStructTopic("RightLimelight", Pose2d.struct).publish();
  StructPublisher<Pose2d> coralLimelightPoseTable =
      visionTable.getStructTopic("CoralLimelight", Pose2d.struct).publish();
  public boolean shouldResetYaw = true;

  private CommandSwerveDrivetrain mDrive;

  public Vision(CommandSwerveDrivetrain mDrive) {
    this.mDrive = mDrive;
  }

  public enum EstimationState {
    NONE,
    TELEOP,
    AUTO,
    AUTO_START,
    TELEOP_YAW;
  }

  private EstimationState estimationState = EstimationState.NONE;

  @Override
  public void periodic() {
    final SwerveDriveState state = mDrive.getState();
    final double rotation = state.Pose.getRotation().getDegrees();
    final double rotationRate = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);

    leftLimelight.setRobotOrientation(rotation, rotationRate);
    PoseEstimate leftLimelightMeasurement = leftLimelight.getPoseEstimate();

    rightLimelight.setRobotOrientation(rotation, rotationRate);
    PoseEstimate rightLimelightMeasurement = rightLimelight.getPoseEstimate();

    switch (this.estimationState) {
      case NONE:
        leftLimelightMeasurement = leftLimelight.getPoseEstimate(false);
        rightLimelightMeasurement = rightLimelight.getPoseEstimate(false);

        addVisionMeasurementForBoth(
            leftLimelightMeasurement,
            rightLimelightMeasurement,
            VisionConstants.YAW_VISION_MEASUREMENT_STANDARD_DEVIATIONS);
        break;

      case AUTO:
      case TELEOP:
        PoseEstimate leftAngle = leftLimelight.getPoseEstimate(false);
        PoseEstimate rightAngle = rightLimelight.getPoseEstimate(false);

        if (isValidMeasurement(leftAngle) && leftAngle.avgTagDist < 1.0) {
          mDrive.addVisionMeasurement(
              leftAngle.pose,
              leftAngle.timestampSeconds,
              Constants.VisionConstants.YAW_VISION_MEASUREMENT_CORRECTION_DEVIATIONS);
        }

        if (isValidMeasurement(rightAngle) && rightAngle.avgTagDist < 1.0) {
          mDrive.addVisionMeasurement(
              rightAngle.pose,
              rightAngle.timestampSeconds,
              Constants.VisionConstants.YAW_VISION_MEASUREMENT_CORRECTION_DEVIATIONS);
        }

        if (Math.abs(rotationRate) < 580.0) {
          addVisionMeasurementForBothWithCalculator(
              leftLimelightMeasurement, rightLimelightMeasurement, EstimationState.TELEOP);
        }
        break;
      default:
    }

    Pose2d leftLimelightPose =
        (leftLimelightMeasurement != null) ? leftLimelightMeasurement.pose : new Pose2d();

    leftLimelightLog.append(leftLimelightPose);
    leftLimelightPoseTable.set(leftLimelightPose);

    Pose2d rightLimelightPose =
        (rightLimelightMeasurement != null) ? rightLimelightMeasurement.pose : new Pose2d();

    rightLimelightLog.append(rightLimelightPose);
    rightLimelightPoseTable.set(rightLimelightPose);

    if (coralLimelight.validTarget()) {
      Pose2d coralLimelightPose = getGamePiecePose();

      coralLimelightPoseTable.set(coralLimelightPose);
    }
  }

  public EstimationState getEstimationState() {
    return estimationState;
  }

  public void setEstimationState(EstimationState newEstimationState) {
    this.estimationState = newEstimationState;
  }

  private boolean isValidMeasurement(PoseEstimate measurement) {
    return measurement != null && measurement.tagCount != 0;
  }

  private void addVisionMeasurement(PoseEstimate measurement, Matrix<N3, N1> standardDeviations) {
    if (isValidMeasurement(measurement)) {
      mDrive.addVisionMeasurement(
          measurement.pose, measurement.timestampSeconds, standardDeviations);
    }
  }

  private void addVisionMeasurementForBoth(
      PoseEstimate leftMeasurement,
      PoseEstimate rightMeasurement,
      Matrix<N3, N1> standardDeviations) {
    addVisionMeasurement(leftMeasurement, standardDeviations);
    addVisionMeasurement(rightMeasurement, standardDeviations);
  }

  private void addVisionMeasurementForBothWithCalculator(
      PoseEstimate leftMeasurement, PoseEstimate rightMeasurement, EstimationState state) {
    if (isValidMeasurement(leftMeasurement)) {
      mDrive.addVisionMeasurement(
          leftMeasurement.pose,
          leftMeasurement.timestampSeconds,
          leftMeasurement.calculateConfidence(state));
    }

    if (isValidMeasurement(rightMeasurement)) {
      mDrive.addVisionMeasurement(
          rightMeasurement.pose,
          rightMeasurement.timestampSeconds,
          rightMeasurement.calculateConfidence(state));
    }
  }

  /**
   * Calculates the pose of the game piece based on Limelight offsets.
   *
   * @param crosshairOffsetX Horizontal offset (tx) in degrees
   * @param crosshairOffsetY Vertical offset (ty) in degrees
   * @param robotPose Current robot pose
   * @return Pose2d of the game piece
   */
  public Pose2d getGamePiecePose(
      double crosshairOffsetX, double crosshairOffsetY, Pose2d robotPose) {
    // Convert angles to radians
    double txRad = Math.toRadians(crosshairOffsetX);
    double tyRad = Math.toRadians(crosshairOffsetY);

    // Calculate distance to the object using camera geometry
    double totalPitch = VisionConstants.kCameraPitchRadians + tyRad; // Should be negative
    double distance = VisionConstants.kCameraHeightMeters * Math.tan(totalPitch);

    double x = distance * Math.cos(txRad); // Forward distance
    double y = distance * Math.sin(txRad); // Sideways offset

    Translation2d gamePieceTranslation =
        new Translation2d(x, y).rotateBy(robotPose.getRotation().plus(new Rotation2d(Math.PI)));

    // Return the pose of the game piece
    return new Pose2d(robotPose.getTranslation().plus(gamePieceTranslation), new Rotation2d());
  }

  public Pose2d getGamePiecePose(double crosshairOffsetX, double crosshairOffsetY) {
    return getGamePiecePose(crosshairOffsetX, crosshairOffsetY, mDrive.getState().Pose);
  }

  public Pose2d getGamePiecePose() {
    return getGamePiecePose(coralLimelight.getTX(), coralLimelight.getTY());
  }

  public boolean seesCoral() {
    return coralLimelight.validTarget();
  }
}
