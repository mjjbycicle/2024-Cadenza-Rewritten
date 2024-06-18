package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
  public static final Pose2d amp_blue = new Pose2d(new Translation2d(), new Rotation2d());
  public static final Pose2d amp_red = new Pose2d(new Translation2d(), new Rotation2d());
  public static final Translation2d speaker_blue = new Translation2d();
  public static final Translation2d speaker_red = new Translation2d();
  public static final Pose2d manualSpeaker_blue = new Pose2d(new Translation2d(), new Rotation2d());
  public static final Pose2d manualSpeaker_red = new Pose2d(new Translation2d(), new Rotation2d());
}
