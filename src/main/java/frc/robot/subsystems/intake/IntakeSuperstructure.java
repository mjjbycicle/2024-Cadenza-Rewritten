package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotInfo.IntakeInfo.*;
import frc.robot.subsystems.intake.IRSensor.IRSensor;
import frc.robot.subsystems.intake.hopper.Hopper;
import frc.robot.subsystems.intake.intake.Intake;

public class IntakeSuperstructure extends SubsystemBase {
  private final Hopper hopper;
  private final Intake intake;
  private final IRSensor irSensor;
  private NoteState state;
  private double noteTime;

  public IntakeSuperstructure(Hopper hopper, Intake intake, IRSensor irSensor) {
    this.hopper = hopper;
    this.intake = intake;
    this.irSensor = irSensor;
    state = NoteState.NOT_POSSESSED;
    noteTime = 0.0;
  }

  @Override
  public void periodic() {
    if (state == NoteState.INTAKING) {
      intake.setExtended(ExtenstionState.EXTENDED);
      intake.intake();
      hopper.intake();
      if (irSensor.hasNote()) {
        noteTime = Timer.getFPGATimestamp();
        state = NoteState.RAW_HOPPER;
        intake.setExtended(ExtenstionState.RETRACTED);
      }
    } else if (state == NoteState.RAW_HOPPER) {
      intake.setExtended(ExtenstionState.RETRACTED);
      hopper.outtake();
      double currTime = Timer.getFPGATimestamp();
      if (currTime - noteTime > 0.12) {
        state = NoteState.LOADED_HOPPER;
      }
    } else if (state == NoteState.SHOOTING) {
      intake.setExtended(ExtenstionState.RETRACTED);
      hopper.intake();
    } else if (state == NoteState.OUTTAKING) {
      intake.setExtended(ExtenstionState.RETRACTED);
      intake.outtake();
      hopper.outtake();
    } else if (state == NoteState.RAW_EXTENDED) {
      intake.setExtended(ExtenstionState.EXTENDED);
      intake.stop();
    } else if (state == NoteState.RAW_RETRACTED) {
      intake.setExtended(ExtenstionState.RETRACTED);
      intake.stop();
    } else {
      intake.setExtended(ExtenstionState.RETRACTED);
      intake.stop();
      hopper.stop();
    }
  }

  public void intake() {
    state = NoteState.INTAKING;
  }

  public void shoot() {
    state = NoteState.SHOOTING;
  }

  public void outtake() {
    state = NoteState.OUTTAKING;
  }

  public boolean loaded() {
    return state == NoteState.LOADED_HOPPER;
  }

  public void stop() {
    state = NoteState.NOT_POSSESSED;
  }

  public void extendIntake() {
    state = NoteState.RAW_EXTENDED;
  }

  public void retractIntake() {
    state = NoteState.RAW_RETRACTED;
  }

  public void toggleIR() {
    irSensor.setActive(!irSensor.isActive());
  }
}
