package frc.robot.subsystems.intake.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  class HopperIOInputs {
    public double hopperVoltage;
  }

  default void set(double percent) {}

  default void updateInputs(HopperIOInputs inputs) {}

  default void stop() {}
}
