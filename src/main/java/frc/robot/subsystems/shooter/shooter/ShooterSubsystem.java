package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.constants.IDs;
import frc.robot.util.InterpolatorTables;
import frc.robot.util.PIDTuner;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX m_shootMotorA, m_shootMotorB;
  private PIDController m_shootPID;
  private double shooterPIDTarget = -1;
  private Double shooterPIDThreshold = null;
  private boolean atTargetSpeedBool = false;
  private Trigger atTargetSpeed;
  private boolean feederRPM = false;

  private PIDTuner shooterTuner = null;

  public ShooterSubsystem() {
    m_shootMotorA = new TalonFX(IDs.ShooterIDs.shooterLeft);
    m_shootMotorB = new TalonFX(IDs.ShooterIDs.shooterRight);

    m_shootMotorA.setInverted(true);
    m_shootMotorA.setNeutralMode(NeutralModeValue.Coast);

    m_shootMotorB.setInverted(false);
    m_shootMotorB.setNeutralMode(NeutralModeValue.Coast);

    m_shootPID = new PIDController(0.001, 0, 0);

    // shooterTuner = new PIDTuner(m_shootPID, ()-> shooterPIDTarget)
    //                   .withName("Shooter")
    //                   .withValue(m_shootEncoder::getVelocity)
    //                   .withOutput(m_shootMotorA::get);
    // shooterTuner.initializeValues(m_shootPID);

    atTargetSpeed = new Trigger(this::isAtTargetSpeed);

    SmartDashboard.putNumber("Shooter Tuning Speed", 3500);
    SmartDashboard.putNumber("Amp Speed", 0.18);

  }

  @Override
  public void periodic() {
    double velocity = m_shootMotorB.getVelocity().getValueAsDouble();

    double thresholdTarget = shooterPIDThreshold != null ? shooterPIDThreshold : shooterPIDTarget;
    if (feederRPM) {
      atTargetSpeedBool = shooterPIDTarget > 0 && Math.abs(thresholdTarget - velocity) <= 125;
    } else {
      atTargetSpeedBool = shooterPIDTarget > 0 &&
              (Math.abs(thresholdTarget - velocity) <= 25 || thresholdTarget < velocity);
    }

    Logger.recordOutput("Shooter RPM", velocity);
    Logger.recordOutput("Shooter RPM Threshold", thresholdTarget);
    Logger.recordOutput("Feeder RPM Mode", feederRPM);
    Logger.recordOutput("At Target Speed?", atTargetSpeedBool);
    Logger.recordOutput("Motors/ShooterA Temperature (C)", m_shootMotorA.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput("Motors/ShooterA Output", m_shootMotorA.getBridgeOutput().getValueAsDouble());
    Logger.recordOutput("Motors/ShooterB Temperature (C)", m_shootMotorB.getDeviceTemp().getValueAsDouble());
    Logger.recordOutput("Motors/ShooterB Output", m_shootMotorB.getBridgeOutput().getValueAsDouble());


    if (shooterTuner != null) shooterTuner.periodic();
  }

  public void runShooterDistanceBased() {
    runShooter(InterpolatorTables.shooterRPMTable().get(RobotContainer.instance.vision.getSpeakerTableKey()));
  }

  /**
   * Run the shooter at the given RPM
   * @param rpm The RPM to run the shooter at
   */
  public void runShooter(double rpm) {
    if (rpm == 0) {
      stopShooter();
      shooterPIDTarget = rpm;
      Logger.recordOutput("Shooter RPM Target", shooterPIDTarget);
      return;
    }
    // Because the Vortexes are on the same shaft and there's latency in the follower CAN frames,
    // both motors running can cause higher oscillation at lower power. So, if we are
    // running at a lower speed, we only want to run one motor.
    if (rpm >= 2000) {
      m_shootMotorB.setControl(new Follower(m_shootMotorA.getDeviceID(), true));
    } else {
      m_shootMotorB.setControl(new EmptyControl());
      m_shootMotorB.setVoltage(0);
    }
    shooterPIDTarget = rpm;
    Logger.recordOutput("Shooter RPM Target", shooterPIDTarget);
  }

  public void runShooterOpenLoop(double speed) {
    m_shootMotorA.setVoltage(speed * 12);
    shooterPIDTarget = -1;
  }

  public Command shooterOpenLoop(double speed) {
    return Commands.runEnd(
            ()-> runShooterOpenLoop(speed),
            ()-> stopShooter()
    );
  }

  public void setShooterThresholdDistanceBased() {
    setShooterThreshold(InterpolatorTables.shooterRPMTable().get(RobotContainer.instance.vision.getSpeakerTableKey()));
  }

  public void setShooterThreshold(Double threshold) {
    shooterPIDThreshold = threshold;
  }

  public void stopShooter(){
    m_shootMotorA.stopMotor();
    m_shootMotorB.setControl(new EmptyControl());
    m_shootMotorB.stopMotor();
    shooterPIDTarget = -1;
  }

  public void setFeederRPM(boolean isFeederRPM) {
    feederRPM = isFeederRPM;
  }

  public boolean isAtTargetSpeed() {
    return atTargetSpeedBool;
  }

  public Trigger atTargetSpeed() {
    return atTargetSpeed;
  }

  public Command setShootSpeedACommand(double speed) {
    return runEnd(
            ()-> {
              m_shootMotorA.set(speed);
            },
            () -> {
              m_shootMotorA.set(0);
            }
    );
  }

  public Command setShootSpeedBCommand(double speed) {
    return runEnd(
            ()-> {
              m_shootMotorB.set(speed);
            },
            () -> {
              m_shootMotorB.set(0);
            }
    );
  }
}
