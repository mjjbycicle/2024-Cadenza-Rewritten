package frc.robot.subsystems.intake.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public double pivotVoltage;
    public double pivotPosition;
  }

  default void setPivot(double percent) {}

  default void setIntake(double percent) {}

  default void updateInputs(IntakeIOInputs inputs) {}

  default void stop() {}

  default void stopIntake() {}

  default double getPosition() {
    return 0;
  }
}
