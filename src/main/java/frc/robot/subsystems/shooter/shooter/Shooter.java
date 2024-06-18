package frc.robot.subsystems.shooter.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotInfo.ShooterInfo;
import math.Averager;

public class Shooter extends SubsystemBase {
  // Components
  private final ShooterIO shooterMotors;
  // Other references
  private final ShooterIOInputsAutoLogged shooterIOInputs = new ShooterIOInputsAutoLogged();
  // Controllers
  private final Averager shooterOutputAverager;
  private final BangBangController bangBangController;
  private double targetSpeed;

  public Shooter(ShooterIO shooter) {
    shooterMotors = shooter;

    shooterOutputAverager = new Averager(Constants.windowLength);

    bangBangController = new BangBangController();
  }

  private double getCurrentSpeed() {
    return shooterIOInputs.shooterSpeed;
  }

  private void applyBangBangControl(double targetSpeed) {
    double currentSpeed = getCurrentSpeed();
    double controllerOutput = bangBangController.calculate(currentSpeed, targetSpeed);

    shooterOutputAverager.addMeasurement(controllerOutput);

    shooterMotors.setShooterVoltage(shooterOutputAverager.getValue() * ShooterInfo.SHOOTER_VOLTAGE);
  }

  @Override
  public void periodic() {
    shooterMotors.updateInputs(shooterIOInputs);
    SmartDashboard.putNumber("shooter speed", getCurrentSpeed());
    applyBangBangControl(targetSpeed);
  }

  public void setTargetSpeed(double speed) {
    this.targetSpeed = speed;
  }

  public boolean isReady() {
    return isUpToSpeed();
  }

  private boolean isUpToSpeed() {
    double rawSpeed = Math.abs(getCurrentSpeed());
    return rawSpeed >= Math.abs(targetSpeed) * 0.97;
  }
}
