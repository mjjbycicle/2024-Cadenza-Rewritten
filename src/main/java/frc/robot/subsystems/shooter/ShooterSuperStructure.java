package frc.robot.subsystems.shooter;

import static frc.robot.constants.RobotInfo.ShooterInfo.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.subsystems.shooter.shooter.Shooter;
import frc.robot.util.AimUtil;

public class ShooterSuperStructure extends SubsystemBase {
  private final Shooter shooter;
  private final Pivot pivot;

  private ShooterMode mode;
  private Pose2d currentPose;

  public ShooterSuperStructure(Shooter shooter, Pivot pivot, Pose2d robotPose) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.mode = ShooterMode.TELEOP_IDLE;
    currentPose = robotPose;
  }

  public ShooterSetpoint getShooterSetpoint(Pose2d robotPose) {
    return switch (mode) {
      case AMP -> SHOOTER_AMP_SETPOINT;
      case MANUAL_SPEAKER -> SHOOTER_SPEAKER_SETPOINT;
      case AUTO_SPEAKER -> AimUtil.get(robotPose);
      case TELEOP_IDLE -> new ShooterSetpoint(
          SHOOTER_TELEOP_IDLE_SPEED, SHOOTER_PIVOT_BOTTOM_SETPOINT);
      case AUTO_IDLE -> new ShooterSetpoint(
          SHOOTER_AUTON_IDLE_SPEED, SHOOTER_PIVOT_BOTTOM_SETPOINT);
      case AUTO_LAUNCH -> SHOOTER_LAUNCH_SETPOINT;
    };
  }

  public void setShootingMode(ShooterMode mode) {
    this.mode = mode;
  }

  public void updatePose(Pose2d robotPose) {
    currentPose = robotPose;
  }

  @Override
  public void periodic() {
    ShooterSetpoint currentSetpoint = getShooterSetpoint(currentPose);
    shooter.setTargetSpeed(currentSetpoint.speed());
    pivot.setAngle(currentSetpoint.angle());
  }

  public boolean ready() {
    return shooter.isReady();
  }
}
