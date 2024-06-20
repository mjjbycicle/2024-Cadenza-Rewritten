package frc.robot.subsystems.intake.IRSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs.IntakeIDs;
import math.Averager;

public class IRSensor extends SubsystemBase {
  private final DigitalInput sensor1;
  private boolean active = true;

  // 0 = clear, 1 = has note
  private final Averager noteDetection = new Averager(5);

  public IRSensor() {
    sensor1 = new DigitalInput(IntakeIDs.IR1);
  }

  public boolean rawSensorIsClear() {
    // Since the IR sensor returns true if it is clear or misaligned or
    // malfunctioning, to make use of the redundancy provided by having two
    // sensors, we use OR so that a misaligned sensor (which always returns
    // true) is equivalent to having no sensor.
    return sensor1.get();
  }

  public void setActive(boolean active) {
    this.active = active;
  }

  public boolean isActive() {
    return this.active;
  }

  public boolean hasNote() {
    return active && noteDetection.getValue() >= 0.5;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IR sensor detects note", !rawSensorIsClear());
    SmartDashboard.putBoolean("IR sensor active", isActive());
    SmartDashboard.putBoolean("IR sensor has note", hasNote());

    noteDetection.addMeasurement(rawSensorIsClear() ? 0 : 1);
  }
}
