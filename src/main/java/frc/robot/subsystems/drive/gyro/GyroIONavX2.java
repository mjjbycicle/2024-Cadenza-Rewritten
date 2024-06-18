package frc.robot.subsystems.drive.gyro;

import frc.robot.subsystems.NavX.NavX;
import frc.robot.subsystems.superstructure.SubsystemManager;

public class GyroIONavX2 implements GyroIO {
  private final NavX navX;

  public GyroIONavX2() {
    navX = SubsystemManager.getNavX();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = navX.getRotation2d();
    inputs.yawVelocityRadPerSec = navX.getRate();
  }
}
