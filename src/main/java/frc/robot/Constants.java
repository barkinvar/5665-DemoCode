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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

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

  public static class VisionConstants {
    public static final String kFrontLimelightTableName = "limelight";
    public static final String kBackLimelightTableName = "limelight-back";

    public static final Translation3d kFrontLimelightTranslation =
        new Translation3d(0.286, -0.286, 0.241);
    public static final Translation3d kBackLimelightTranslation =
        new Translation3d(-0.286, -0.286, 0.241);

    public static final Rotation3d kFrontLimelightRotation =
        new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(10.0));
    public static final Rotation3d kBackLimelightRotation =
        new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(170.0));

    public static final AprilTagFieldLayout kAprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final List<AprilTag> tagList = kAprilTagLayout.getTags();
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class FieldConstants {
    public static final Translation2d redReefCenterPos = new Translation2d(13.08, 4.02);
  }

  public static class ElevatorConstans {
    public static final int kLeftMotorID = 0;
    public static final int kRightMotorID = 1;

    public static final double kRatio = 10.0; // TODO

    public static final double kP = 0.0;
    public static final double kD = 0.0;

    public static final double currentLimit = 10.0;

    public static final double kTolerance = 2.0; // TODO
  }
}
