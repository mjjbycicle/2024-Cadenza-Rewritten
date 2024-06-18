package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.oi.OI;
import frc.robot.subsystems.superstructure.SubsystemManager;

public class MovementUtil {
  private static Translation2d desiredPose, targetLock;
  private static Rotation2d desiredRotation;
  private static boolean toDesired, locked, useRotation;
  public static boolean configured = false;

  private static final OI oi = SubsystemManager.getOI();

  private static final PIDController rotationPID = new PIDController(0.08, 0, 0);
  private final PIDController movementXPID = new PIDController(0.5, 0, 0);
  private final PIDController movementYPID = new PIDController(0.5, 0, 0);

  public static void configure(
          Translation2d desiredPose,
          Translation2d targetLock,
          Rotation2d desiredRotation,
          boolean toDesired,
          boolean locked,
          boolean useRotation) {
    MovementUtil.desiredPose = desiredPose;
    MovementUtil.targetLock = targetLock;
    MovementUtil.desiredRotation = desiredRotation;
    MovementUtil.toDesired = toDesired;
    MovementUtil.locked = locked;
    MovementUtil.useRotation = useRotation;
    configured = true;
  }

  public static void setLocked(Translation2d targetLock) {
    this.locked = true;
    this.useRotation = false;
    this.targetLock = targetLock;
  }

  public static void setLocked(Rotation2d desiredRotation) {
    this.locked = true;
    this.useRotation = true;
    this.desiredRotation = desiredRotation;
  }

  public static void setLocked(boolean locked) {
    this.locked = locked;
  }

  public static void setToDesired(Translation2d desiredPose) {
    this.toDesired = true;
    this.desiredPose = desiredPose;
  }

  public static void setToDesired(boolean toDesired) {
    this.toDesired = toDesired;
  }

  public Rotation2d getDesiredRotation(Pose2d robotPose) {
    if (useRotation) {
      return desiredRotation;
    } else {
      Translation2d currTranslation = robotPose.getTranslation();
      Translation2d robotToPose = currTranslation.minus(targetLock);
      if (DriverStationUtil.isRed()) {
        robotToPose = new Translation2d().minus(robotToPose);
      }
      double x = robotToPose.getX();
      double y = robotToPose.getY();
      return Rotation2d.fromRadians(Math.atan(y / x));
    }
  }

  public Translation2d getPoseOffset(Pose2d robotPose) {
    Translation2d currTranslation = robotPose.getTranslation();
    double currX = currTranslation.getX();
    double currY = currTranslation.getY();
    double targetX = desiredPose.getX();
    double targetY = desiredPose.getY();
    double xDiff, yDiff;
    if (DriverStationUtil.isRed()) {
      xDiff = currX - targetX;
      yDiff = targetY - currY;
    } else {
      xDiff = targetX - currX;
      yDiff = currY - targetY;
    }
    return new Translation2d(xDiff, yDiff);
  }

  public static ChassisSpeeds getRobotRelative(Pose2d robotPose) {
    Rotation2d desiredRotation = getDesiredRotation(robotPose);
    Translation2d translationDiff = getPoseOffset(robotPose);
    OI.AxesInputs driveInputs = oi.getLeftStickInputs();
    OI.AxesInputs turnInputs = oi.getRightStickInputs();
    double rotationSpeed;
    if (locked) {
      rotationPID.setSetpoint(desiredRotation.getDegrees());
      rotationSpeed = rotationPID.calculate(desiredRotation.getDegrees());
    } else {
      rotationSpeed = -turnInputs.x;
    }
    double xSpeed, ySpeed;
    if (toDesired) {
      movementXPID.setSetpoint(0);
      movementYPID.setSetpoint(0);
      xSpeed = movementXPID.calculate(translationDiff.getX());
      ySpeed = movementYPID.calculate(translationDiff.getY());
    } else {
      xSpeed = driveInputs.x;
      ySpeed = driveInputs.y;
    }
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        ySpeed, xSpeed, rotationSpeed, robotPose.getRotation());
  }

  public static void reset() {
    locked = false;
    toDesired = false;
  }

  public boolean atTranslation(Pose2d robotPose) {
    Translation2d currDiff = getPoseOffset(robotPose);
    double x = Math.abs(currDiff.getX());
    double y = Math.abs(currDiff.getY());
    return x < 0.02 && y < 0.02;
  }

  public boolean atRotation(Pose2d robotPose) {
    double rotationDiffDegrees =
        Math.abs(desiredRotation.minus(robotPose.getRotation()).getDegrees());
    return rotationDiffDegrees < 5;
  }

  public boolean atPose(Pose2d robotPose) {
    if (toDesired && locked) {
      return atTranslation(robotPose) && atRotation(robotPose);
    } else {
      return false;
    }
  }
}
