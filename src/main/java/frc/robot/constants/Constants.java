package frc.robot.constants;

public class Constants {
  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final Mode currentMode = Mode.REAL;

  public static final int windowLength = 1;
}
