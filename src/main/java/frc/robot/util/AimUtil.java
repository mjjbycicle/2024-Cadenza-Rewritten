package frc.robot.util;

import static frc.robot.constants.FieldConstants.*;
import static frc.robot.constants.ShooterSetpoints.measurements;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotInfo.ShooterInfo.ShooterSetpoint;
import frc.robot.constants.RobotInfo.ShooterInfo.ShooterSetpointMeasurement;
import math.MathUtil;

public class AimUtil {
  public static ShooterSetpoint get(double distance) {

    SmartDashboard.putNumber("Limelight distance: ", distance);

    ShooterSetpointMeasurement a = measurements[0], b = measurements[1];
    for (int i = 1; i < measurements.length; i++) {
      a = measurements[i - 1];
      b = measurements[i];
      if (a.distance() < distance && distance < b.distance()) {
        break;
      }
    }

    // Lerp
    double t = (distance - a.distance()) / (b.distance() - a.distance());
    t = MathUtil.clamp(t, 0, 1);

    return new ShooterSetpoint(
        MathUtil.lerp(a.setpoint().speed(), b.setpoint().speed(), t),
        MathUtil.lerp(a.setpoint().angle(), b.setpoint().angle(), t));
  }

  public static ShooterSetpoint get(Pose2d botPose) {
    Translation2d speaker;
    if (DriverStationUtil.isRed()) {
      speaker = speaker_red;
    } else {
      speaker = speaker_blue;
    }
    Translation2d vec = speaker.minus(botPose.getTranslation());
    double x = vec.getX();
    double y = vec.getY();
    double distance = Math.hypot(x, y);
    return get(distance);
  }
}
