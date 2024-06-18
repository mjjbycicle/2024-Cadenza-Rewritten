package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotInfo.ShooterInfo;

public class Pivot extends SubsystemBase {
  // Components
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs;
  // Control
  private final PIDController aimPID;
  private boolean isUsingPID = true;

  private double targetAngle;

  public Pivot(PivotIO shooterPivotIO) {
    this.pivotIO = shooterPivotIO;
    this.pivotInputs = new PivotIOInputsAutoLogged();

    aimPID = ShooterInfo.SHOOTER_AIM_PID_CONSTANTS.create();
    aimPID.setSetpoint(ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT);

    targetAngle = ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT;
  }

  public boolean isAtSetPoint() {
    double error = Math.abs(aimPID.getSetpoint() - pivotIO.getPosition());
    return error < ShooterInfo.SHOOTER_PIVOT_ERROR;
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    if (!isUsingPID) return;

    aimPID.setSetpoint(targetAngle);

    double currentAngle = getCurrentAngle();
    double pidOutput = aimPID.calculate(currentAngle);

    pivotIO.setPivot(pidOutput);

    SmartDashboard.putNumber("CurrentShooterAngle", currentAngle);
    SmartDashboard.putNumber("TargetShooterAngle", targetAngle);
    SmartDashboard.putNumber("AimPIDOutput", pidOutput);
  }

  private double getCurrentAngle() {
    return pivotIO.getPosition();
  }

  public void stop() {
    pivotIO.stop();
  }

  public void setAngle(double angle) {
    targetAngle = angle;
  }
}
