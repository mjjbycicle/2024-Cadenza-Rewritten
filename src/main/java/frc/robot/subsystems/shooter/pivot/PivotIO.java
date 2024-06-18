package frc.robot.subsystems.shooter.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  class PivotIOInputs {
    public double pivotVoltage;
    public double pivotPositon;
  }

  default void setPivot(double percent) {}

  default void updateInputs(PivotIOInputs inputs) {}

  default void stop() {}

  default double getPosition() {
    return 0;
  }
}
