package frc.robot.subsystems.NavX;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Encapsulates the NavX on the robot. Use this class for determining the angle of the robot. */
public class NavX extends SubsystemBase {
  private final AHRS ahrs;
  private double prevHeading;
  private double prevRate;
  private double offset = 0;

  public NavX() {
    ahrs = new AHRS(SPI.Port.kMXP);
    offset = ahrs.getRotation2d().getRadians();
  }

  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  public double getRoll() {
    return ahrs.getRoll();
  }

  public double getPitch() {
    return ahrs.getPitch();
  }

  public Rotation2d getRotation2d() {
    return ahrs.getRotation2d().minus(new Rotation2d(offset));
  }

  public void reset() {
    offset = ahrs.getRotation2d().getRadians();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("heading", "%.2fdeg".formatted(getRotation2d().getDegrees()));
    SmartDashboard.putString("matchTime", "%d".formatted((int) Timer.getMatchTime()));
    if (getRate() != 0) {
      SmartDashboard.putNumber("rate", getRate());
      prevRate = getRate();
    } else {
      SmartDashboard.putNumber("rate", prevRate);
    }
    prevHeading = getRotation2d().getDegrees();
    SmartDashboard.putNumber("offset", offset);
  }

  public double getRate() {
    return Math.abs(getRotation2d().getDegrees() - prevHeading) * 50;
  }
}
