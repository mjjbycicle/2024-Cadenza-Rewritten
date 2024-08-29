package frc.robot.constants;

public class Constants {
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.508;

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final Mode currentMode = Mode.REAL;

  public static final int windowLength = 1;

  public static final double FIELD_WIDTH_METERS = 16.5410515;

  public static final double LOOP_TIME = 0.020;
}
