package frc.robot.subsystems.intake.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.IDs.*;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax intakePivotLeft;
  private final CANSparkMax intakePivotRight;
  private final CANSparkMax intakeMotor;
  private final DutyCycleEncoder encoder;

  public IntakeIOSparkMax() {
    intakePivotLeft = new CANSparkMax(IntakeIDs.pivotLeft, CANSparkMax.MotorType.kBrushless);
    intakePivotLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);

    intakePivotRight = new CANSparkMax(IntakeIDs.pivotRight, CANSparkMax.MotorType.kBrushless);
    intakePivotRight.follow(intakePivotLeft, true);

    intakeMotor = new CANSparkMax(IntakeIDs.intakeMotor, CANSparkMax.MotorType.kBrushless);

    encoder = new DutyCycleEncoder(IntakeIDs.intakeEncoder);
  }

  @Override
  public void setPivot(double percent) {
    intakePivotLeft.setVoltage(percent);
  }

  @Override
  public void setIntake(double percent) {
    intakeMotor.setVoltage(percent);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.pivotVoltage = intakePivotLeft.getBusVoltage();
    inputs.pivotPosition = encoder.getAbsolutePosition();
  }

  @Override
  public void stop() {
    intakePivotLeft.stopMotor();
    intakeMotor.stopMotor();
  }

  @Override
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  @Override
  public double getPosition() {
    return encoder.getAbsolutePosition();
  }
}
