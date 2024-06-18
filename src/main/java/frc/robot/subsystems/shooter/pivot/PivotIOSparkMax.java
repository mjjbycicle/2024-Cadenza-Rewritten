package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.constants.IDs.*;

public class PivotIOSparkMax implements PivotIO {
  private final CANSparkMax pivotNeoLeft;
  private final CANSparkMax pivotNeoRight;
  private final DutyCycleEncoder encoder;

  public PivotIOSparkMax() {
    pivotNeoLeft = new CANSparkMax(ShooterIDs.pivotLeft, CANSparkMax.MotorType.kBrushless);
    pivotNeoLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);

    pivotNeoRight = new CANSparkMax(ShooterIDs.pivotRight, CANSparkLowLevel.MotorType.kBrushless);
    pivotNeoRight.follow(pivotNeoLeft, true);

    encoder = new DutyCycleEncoder(ShooterIDs.shooterEncoder);
  }

  @Override
  public void setPivot(double percent) {
    pivotNeoLeft.setVoltage(percent);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotVoltage = pivotNeoLeft.getBusVoltage();
  }

  @Override
  public void stop() {
    pivotNeoLeft.stopMotor();
  }

  @Override
  public double getPosition() {
    return encoder.getAbsolutePosition();
  }
}
