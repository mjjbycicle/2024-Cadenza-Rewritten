package frc.robot.constants;

import static frc.robot.constants.ShooterSetpoints.measurements;

public class RobotInfo {
  public static class DriveInfo {
    public enum DriveMode {
      DRIVE_TO_AMP,
      LOCK_TO_AMP,
      LOCK_TO_SPEAKER,
      MANUAL_DRIVE
    }
  }

  public static class ShooterInfo {

    public record ShooterSetpoint(double speed, double angle) {}

    public record ShooterSetpointMeasurement(
        double distance, ShooterInfo.ShooterSetpoint setpoint) {}

    public static final double SHOOTER_TELEOP_IDLE_SPEED = 0.2;
    public static final double SHOOTER_AUTON_IDLE_SPEED = 0.35;
    public static final double SHOOTER_PIVOT_BOTTOM_SETPOINT = .579;

    public static final double SHOOTER_PIVOT_ERROR = 0.01;

    public static final ShooterInfo.ShooterSetpoint SHOOTER_AMP_SETPOINT =
        new ShooterInfo.ShooterSetpoint(6.8, .49);
    public static final ShooterInfo.ShooterSetpoint SHOOTER_SPEAKER_SETPOINT =
        measurements[1].setpoint;
    public static final ShooterInfo.ShooterSetpoint SHOOTER_TRAP_SETPOINT =
        new ShooterInfo.ShooterSetpoint(20, .499);
    public static final ShooterInfo.ShooterSetpoint SHOOTER_LAUNCH_SETPOINT =
        new ShooterInfo.ShooterSetpoint(40, .529);

    public static final ShooterInfo.ShooterSetpoint SHOOTER_INTAKE_SETPOINT =
        new ShooterInfo.ShooterSetpoint(-15, .529);

    public static final double SHOOTER_VOLTAGE = 10;

    public static final PIDTemplate SHOOTER_AIM_PID_CONSTANTS = new PIDTemplate(4, 0, 0);

    public enum ShooterMode {
      AMP,
      MANUAL_SPEAKER,
      AUTO_SPEAKER,
      TELEOP_IDLE,
      AUTO_IDLE,
      AUTO_LAUNCH
    }
  }

  public static class IntakeInfo {
    public enum IntakeMode {
      INTAKING,
      OUTTAKING
    }

    public enum ExtenstionState {
      EXTENDED,
      RETRACTED
    }

    public static final double intakeSpeed = -0.85;
    public static final double intakeRetractedSetpoint = 0.63;
    public static final double intakeExtendedSetpoint = 0.87;
    public static final PIDTemplate intakePID = new PIDTemplate(.8, 0, 0);
    public static final double intakeUpMultiplier = 2;
    public static final double hopperSpeed = 0.8; // was 0.7
  }
}
