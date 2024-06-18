package frc.robot.subsystems.intake.intake;

import static frc.robot.constants.RobotInfo.IntakeInfo.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs;
  private final PIDController pivotPID;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    pivotPID = intakePID.create();
    inputs = new IntakeIOInputsAutoLogged();
    setExtended(ExtenstionState.RETRACTED);
  }

  public void setExtended(ExtenstionState extended) {
    pivotPID.setSetpoint(
        switch (extended) {
          case EXTENDED -> intakeExtendedSetpoint;
          case RETRACTED -> intakeRetractedSetpoint;
        });
  }

  // Spins intake motor to intake notes
  public void intake() {
    intakeIO.setIntake(intakeSpeed);
  }

  public void outtake() {
    intakeIO.setIntake(-intakeSpeed);
  }

  @Override
  public void periodic() {
    double pidOutput = -pivotPID.calculate(intakeIO.getPosition());
    if (pidOutput > 0) pidOutput *= intakeUpMultiplier;
    intakeIO.setPivot(pidOutput);
    intakeIO.updateInputs(inputs);
  }

  public void stop() {
    intakeIO.stop();
  }

  public void stopIntake() {
    intakeIO.stopIntake();
  }
}
