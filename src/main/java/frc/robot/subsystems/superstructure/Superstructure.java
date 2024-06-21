package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotInfo.DriveInfo.*;
import frc.robot.constants.RobotInfo.ShooterInfo.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.shooter.ShooterSuperStructure;
import frc.robot.util.ControllerUtil;

public class Superstructure extends SubsystemBase {
  private final Drive drive;
  private final ShooterSuperStructure shooter;
  private final IntakeSuperstructure intake;

  private Action currentAction;

  private Pose2d currentPose;

  public enum Action {
    AUTO_AMP,
    MOVING_SPEAKER,
    MANUAL_SPEAKER,
    MANUAL_INTAKE,
    MANUAL_DRIVE,
    AUTO_LAUNCH,
    LAUNCH_SPINUP,
    OUTTAKE,
    EXTEND_INTAKE,
    RETRACT_INTAKE
  }

  public Superstructure() {
    drive = SubsystemManager.getDrive();
    shooter = SubsystemManager.getShooter();
    intake = SubsystemManager.getIntake();
    currentPose = drive.getPose();
  }

  @Override
  public void periodic() {
    currentPose = drive.getPose();
    shooter.updatePose(currentPose);
    runDriveAction();
    runShooterAction();
    runIntakeAction();
  }

  public void runShooterAction() {
    switch (currentAction) {
      case AUTO_AMP:
        if (ControllerUtil.getAmpShot()) {
          shootAmp();
        }
      case MOVING_SPEAKER:
        if (ControllerUtil.getSpeakerShot()) {
          shootSpeaker();
        }
      case MANUAL_SPEAKER:
        manualSpeaker();
      case LAUNCH_SPINUP, AUTO_LAUNCH:
        launch();
      default:
        shooterIdle();
    }
  }

  public void runIntakeAction() {
    switch (currentAction) {
      case AUTO_AMP, MOVING_SPEAKER, MANUAL_SPEAKER, AUTO_LAUNCH:
        hopperShoot();
      case MANUAL_INTAKE:
        intake();
      case OUTTAKE:
        outtake();
      case EXTEND_INTAKE:
        extendIntake();
      case RETRACT_INTAKE:
        retractIntake();
      default:
        stopIntake();
    }
  }

  public void runDriveAction() {
    if (ControllerUtil.idle()) {
      currentAction = Action.MANUAL_DRIVE;
    }
    switch (currentAction) {
      case AUTO_AMP:
        driveToAmp();
      case MOVING_SPEAKER:
        lockToSpeaker();
      case LAUNCH_SPINUP:
        lockToAmp();
      case AUTO_LAUNCH:
        lockToAmp();
      default:
        manualDrive();
    }
  }

  public void driveToAmp() {
    drive.setDriveMode(DriveMode.DRIVE_TO_AMP);
  }

  public void driveToSpeaker() {
    drive.setDriveMode(DriveMode.DRIVE_TO_SPEAKER);
  }

  public void lockToSpeaker() {
    drive.setDriveMode(DriveMode.LOCK_TO_SPEAKER);
  }

  public void lockToAmp() {
    drive.setDriveMode(DriveMode.LOCK_TO_AMP);
  }

  public void manualDrive() {
    drive.setDriveMode(DriveMode.MANUAL_DRIVE);
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

  public void shooterIdle() {
    shooter.setShootingMode(ShooterMode.TELEOP_IDLE);
  }

  public void intake() {
    intake.intake();
  }

  public void outtake() {
    intake.outtake();
  }

  public void hopperShoot() {
    intake.shoot();
  }

  public void stopIntake() {
    intake.stop();
  }

  public void extendIntake() {
    intake.extendIntake();
  }

  public void retractIntake() {
    intake.retractIntake();
  }

  public Command extendIntakeCommand() {
    return new RunCommand(() -> currentAction = Action.EXTEND_INTAKE, intake, drive, shooter);
  }

  public Command retractIntakeCommand() {
    return new RunCommand(() -> currentAction = Action.RETRACT_INTAKE, intake, drive, shooter);
  }

  public Command intakeCommand() {
    return new RunCommand(() -> currentAction = Action.MANUAL_INTAKE, intake, drive, shooter);
  }

  public Command outtakeCommand() {
    return new RunCommand(() -> currentAction = Action.OUTTAKE, intake, drive, shooter);
  }

  public Command movingSpeakerCommand() {
    return new RunCommand(() -> currentAction = Action.MOVING_SPEAKER, intake, drive, shooter);
  }

  public Command autoAmpCommand() {
    return new RunCommand(() -> currentAction = Action.AUTO_AMP, intake, drive, shooter);
  }

  public Command autoLaunchCommand() {
    return new RunCommand(() -> currentAction = Action.AUTO_LAUNCH, intake, drive, shooter);
  }

  public Command launchSpinupCommand() {
    return new RunCommand(() -> currentAction = Action.LAUNCH_SPINUP, intake, drive, shooter);
  }

  public Command manualSpeakerCommand() {
    return new RunCommand(() -> currentAction = Action.MANUAL_SPEAKER, intake, drive, shooter);
  }

  public Command manualDriveCommand() {
    return new RunCommand(() -> currentAction = Action.MANUAL_DRIVE, intake, drive, shooter);
  }

  public Command toggleIRCommand() {
    return new RunCommand(intake::toggleIR);
  }
}
