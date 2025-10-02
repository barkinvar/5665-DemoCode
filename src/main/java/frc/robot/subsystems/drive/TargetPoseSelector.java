package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TargetPoseSelector {
  private static final double DEFAULT_CLOSEST_DISTANCE = 20.0;

  private CommandSwerveDrivetrain mDrive;
  private Pose2d targetPose = new Pose2d();
  private Pose2d targetHP = new Pose2d();
  private double closestDistance = DEFAULT_CLOSEST_DISTANCE;
  private boolean isAlgeaHigh = false;
  private boolean isLeft = true;

  private DataLog log = DataLogManager.getLog();
  private final StructLogEntry<Pose2d> targetReefLog =
      StructLogEntry.create(log, "TargetPoseSelector/TargetReef", Pose2d.struct);
  private final StructLogEntry<Pose2d> targetHPLog =
      StructLogEntry.create(log, "TargetPoseSelector/TargetHP", Pose2d.struct);
  private final BooleanLogEntry isAlgeaHighLog =
      new BooleanLogEntry(log, "TargetPoseSelector/IsAlgeaHigh");

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable driveTargetTable = inst.getTable("TargetPoseSelector");
  private final StructPublisher<Pose2d> targetReefTopic =
      driveTargetTable.getStructTopic("TargetReef", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> targetHPTopic =
      driveTargetTable.getStructTopic("TargetHP", Pose2d.struct).publish();
  private final BooleanPublisher isAlgeaHighTopic =
      driveTargetTable.getBooleanTopic("IsAlgeaHigh").publish();

  public enum TargetPose {
    LEFT,
    CENTER,
    RIGHT,
    AUTO
  }

  public TargetPoseSelector(CommandSwerveDrivetrain drive){
    this.mDrive = drive;
  }

  private Pose2d[] getTargetPositionsBasedOnAlliance() {
    Alliance currentAlliance = mDrive.getAlliance();
    return (currentAlliance == Alliance.Blue)
        ? FieldConstants.kReefBlueTargets
        : FieldConstants.kReefRedTargets;
  }

  private Pose2d[] getHPsBasedOnAlliance() {
    Alliance currentAlliance = mDrive.getAlliance();
    return (currentAlliance == Alliance.Blue)
        ? FieldConstants.kHPBlueTargets
        : FieldConstants.kHPRedTargets;
  }

  private double calculateDistance(Pose2d current, Pose2d target) {
    return current.relativeTo(target).getTranslation().getNorm();
  }

  public void setTargetHack(TargetPose targetType) {
    closestDistance = DEFAULT_CLOSEST_DISTANCE;
    Pose2d shifted =
        mDrive.getAlliance() == Alliance.Red
            ? new Pose2d(14.6, 4.0, new Rotation2d())
            : new Pose2d(2.6, 4.0, new Rotation2d());
    Pose2d[] targets = getTargetPositionsBasedOnAlliance();

    int targetIndex = findClosestTarget(shifted, targets);
    int targetPoseIndex;

    switch (targetType) {
      case AUTO:
        targetPoseIndex = getAutoTargetPoseIndex(shifted, targets, targetIndex);
        isLeft = targetPoseIndex % 3 == 0;
        break;
      case LEFT:
        targetPoseIndex = targetIndex - 1;
        isLeft = true;
        break;
      case RIGHT:
        targetPoseIndex = targetIndex + 1;
        isLeft = false;
        break;
      default:
        targetPoseIndex = targetIndex;
        break;
    }
    targetPose = targets[targetPoseIndex];

    isAlgeaHigh = targetPoseIndex % 2 != 0;

    targetReefLog.append(targetPose);
    targetReefTopic.set(targetPose);

    isAlgeaHighLog.append(isAlgeaHigh);
    isAlgeaHighTopic.set(isAlgeaHigh);
  }

  public void updateTargetNearest(TargetPose targetType) {
    closestDistance = DEFAULT_CLOSEST_DISTANCE;
    SwerveDriveState currentState = mDrive.getState();
    Pose2d shifted = currentState.Pose.exp(currentState.Speeds.toTwist2d(0.225)); // TODO
    Pose2d[] targets = getTargetPositionsBasedOnAlliance();

    int targetIndex = findClosestTarget(shifted, targets);
    int targetPoseIndex;

    switch (targetType) {
      case AUTO:
        targetPoseIndex = getAutoTargetPoseIndex(shifted, targets, targetIndex);
        isLeft = targetPoseIndex % 3 == 0;
        break;
      case LEFT:
        targetPoseIndex = targetIndex - 1;
        isLeft = true;
        break;
      case RIGHT:
        targetPoseIndex = targetIndex + 1;
        isLeft = false;
        break;
      default:
        targetPoseIndex = targetIndex;
        break;
    }
    targetPose = targets[targetPoseIndex];

    isAlgeaHigh = targetPoseIndex % 2 != 0;

    targetReefLog.append(targetPose);
    targetReefTopic.set(targetPose);

    isAlgeaHighLog.append(isAlgeaHigh);
    isAlgeaHighTopic.set(isAlgeaHigh);
  }

  public void updateHPNearest() {
    closestDistance = DEFAULT_CLOSEST_DISTANCE;
    Pose2d currentPose = mDrive.getState().Pose;
    Pose2d[] targets = getHPsBasedOnAlliance();

    double distanceLeft = calculateDistance(currentPose, targets[0]);
    double distanceRight = calculateDistance(currentPose, targets[1]);

    targetHP = distanceLeft > distanceRight ? targets[1] : targets[0];

    targetHPLog.append(targetHP);
    targetHPTopic.set(targetHP);
  }

  private int findClosestTarget(Pose2d currentPose, Pose2d[] targets) {
    int targetIndex = 0;
    for (int i = 1; i < targets.length; i += 3) {
      double distance = calculateDistance(currentPose, targets[i]);
      if (distance < closestDistance) {
        closestDistance = distance;
        targetIndex = i;
      }
    }
    return targetIndex;
  }

  private int getAutoTargetPoseIndex(Pose2d currentPose, Pose2d[] targets, int targetIndex) {
    double leftDistance = calculateDistance(currentPose, targets[targetIndex - 1]);
    double rightDistance = calculateDistance(currentPose, targets[targetIndex + 1]);

    return (leftDistance < rightDistance) ? targetIndex - 1 : targetIndex + 1;
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public Pose2d getTargetHP() {
    return targetHP;
  }

  public boolean canTurnAtTime(boolean isFar) {
    Pose2d currentPose = mDrive.getState().Pose;
    Pose2d difference = currentPose.relativeTo(targetPose);

    double distance = difference.getTranslation().getNorm();
    double rotation = difference.getRotation().getDegrees();

    return Math.abs(rotation) < 2.0
        || Math.abs(rotation * DriveConstants.kTurnClearanceRatio)
            < (distance + (isFar ? 0.12 : 0.0));
  }

  public boolean getIsAlgeaHigh() {
    return isAlgeaHigh;
  }

  public boolean getIsLeft() {
    return isLeft;
  }
}
