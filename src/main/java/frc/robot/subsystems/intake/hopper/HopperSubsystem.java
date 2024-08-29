package frc.robot.subsystems.intake.hopper;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;

public class HopperSubsystem extends SubsystemBase {
  private final CANSparkMax motor;

  public HopperSubsystem() {
    motor = new CANSparkMax(IDs.IntakeIDs.hopperMotor, CANSparkMax.MotorType.kBrushed);
    motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
  }

  public void run(double speed) {
    motor.set(speed);
  }

  public void run(double speed, boolean reversed) {
    if (reversed) {
      run(-speed);
    } else {
      run(speed);
    }
  }

  public void runBackwards(double speed) {
    motor.set(-speed);
  }

  public void stop() {
    motor.stopMotor();
  }
}
