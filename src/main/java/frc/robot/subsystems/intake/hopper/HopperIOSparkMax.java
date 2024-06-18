package frc.robot.subsystems.intake.hopper;

import com.revrobotics.CANSparkMax;
import frc.robot.constants.IDs.IntakeIDs;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class HopperIOSparkMax implements HopperIO {

  private final CANSparkMax hopperMotor;

  public HopperIOSparkMax() {
    hopperMotor = new CANSparkMax(IntakeIDs.hopperMotor, CANSparkMax.MotorType.kBrushed);
  }

  @Override
  public void set(double percent) {
    hopperMotor.set(percent);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.hopperVoltage = hopperMotor.getBusVoltage();
  }

  @Override
  public void stop() {
    hopperMotor.stopMotor();
  }
}
