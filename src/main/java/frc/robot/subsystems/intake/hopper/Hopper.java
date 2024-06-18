package frc.robot.subsystems.intake.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotInfo.IntakeInfo;

public class Hopper extends SubsystemBase {
  private final HopperIO hopperIO;
  private final HopperIOInputsAutoLogged inputs;

  public Hopper(HopperIO hopperIO) {
    this.hopperIO = hopperIO;
    inputs = new HopperIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    hopperIO.updateInputs(inputs);
  }

  public void intake() {
    hopperIO.set(IntakeInfo.hopperSpeed);
  }

  public void outtake() {
    hopperIO.set(-IntakeInfo.hopperSpeed);
  }

  public void stop() {
    hopperIO.stop();
  }
}
