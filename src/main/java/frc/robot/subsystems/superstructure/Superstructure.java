package frc.robot.subsystems.superstructure;

import static frc.robot.constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotInfo.DriveInfo.*;
import frc.robot.constants.RobotInfo.IntakeInfo.*;
import frc.robot.constants.RobotInfo.ShooterInfo.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterSuperStructure;
import frc.robot.util.DriverStationUtil;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final ShooterSuperStructure shooter;

  private ShooterMode shootingMode;
  private DriveMode driveMode;
  private IntakeMode intakeMode;

  private final Pose2d amp;
  private final Translation2d speaker;

  private Pose2d currentPose;

  private enum Action {
    AUTO_AMP,
    MOVING_SPEAKER,
    MANUAL_SPEAKER,
    MANUAL_INTAKE
  }

  public Superstructure() {
    drive = SubsystemManager.getDrive();
    shooter = SubsystemManager.getShooter();
    if (DriverStationUtil.isRed()) {
      amp = amp_red;
      speaker = speaker_red;
    } else {
      amp = amp_blue;
      speaker = speaker_blue;
    }
    currentPose = drive.getPose();
  }

  @Override
  public void periodic() {
    currentPose = drive.getPose();
    shooter.updatePose(currentPose);
  }

  public void runDriveState() {
    switch (driveMode) {
      case DRIVE_TO_AMP:
        driveToPose(amp);
        break;
      case LOCK_TO_AMP:
        driveLocked(amp.getTranslation());
        break;
      case LOCK_TO_SPEAKER:
        driveLocked(speaker);
      case MANUAL_DRIVE:
        driveManual();
    }
  }

  public void runShooterState() {
    switch (shootingMode) {
      case AMP:
        shootAmp();
        break;
      case AUTO_SPEAKER:
        shootSpeaker();
        break;
      case MANUAL_SPEAKER:
        manualSpeaker();
        break;
      case AUTO_LAUNCH:
        launch();
        break;
    }
  }

  public void driveLocked(Translation2d pose) {
    drive.resetMovement();
    drive.setLocked(pose);
  }

  public void driveManual() {
    drive.resetMovement();
  }

  public void driveTo(Translation2d pose) {
    drive.resetMovement();
    drive.setToDesired(pose);
  }

  public void driveToPose(Pose2d targetPose) {
    drive.resetMovement();
    drive.setLocked(targetPose.getRotation());
    drive.setToDesired(targetPose.getTranslation());
  }

  public void shootSpeaker() {
    shooter.setShootingMode(ShooterMode.AUTO_SPEAKER);
  }

  public void manualSpeaker() {
    shooter.setShootingMode(ShooterMode.MANUAL_SPEAKER);
  }

  public void shootAmp() {
    shooter.setShootingMode(ShooterMode.AMP);
  }

  public void launch() {
    shooter.setShootingMode(ShooterMode.AUTO_LAUNCH);
  }
}
