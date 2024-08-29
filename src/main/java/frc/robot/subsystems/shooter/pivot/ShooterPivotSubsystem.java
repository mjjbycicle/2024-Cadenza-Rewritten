package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.IDs;
import frc.robot.util.Helpers;
import frc.robot.util.InterpolatorTables;
import frc.robot.util.PIDTuner;
import org.littletonrobotics.junction.Logger;

public class ShooterPivotSubsystem extends SubsystemBase {

  public static final double FEED_ANGLE = 235; // was 227.5, 230, low shot 200;
  public static final double TRAP_ANGLE = 247;

  private final double pivotMin = 198.453;
  private final double pivotMax = 257.5;

  private CANSparkMax m_pivotMotorLeft, m_pivotMotorRight;
  private SparkPIDController m_pivotPID;
  private SparkAbsoluteEncoder m_pivotEncoder;
  private double pivotPIDTarget = -1;
  private double position = 0;

  private PIDTuner pivotTuner = null;

  public ShooterPivotSubsystem() {
    m_pivotMotorLeft = new CANSparkMax(IDs.ShooterIDs.pivotLeft, MotorType.kBrushless);
    m_pivotMotorLeft.restoreFactoryDefaults();
    Helpers.setSparkInverted(m_pivotMotorLeft, true);
    m_pivotMotorLeft.setIdleMode(IdleMode.kBrake);


    m_pivotMotorRight = new CANSparkMax(IDs.ShooterIDs.pivotRight, MotorType.kBrushless);
    m_pivotMotorRight.restoreFactoryDefaults();
    m_pivotMotorRight.setIdleMode(IdleMode.kBrake);
    m_pivotMotorRight.follow(m_pivotMotorLeft, true);

    m_pivotEncoder = m_pivotMotorLeft.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_pivotEncoder.setPositionConversionFactor(360);
    m_pivotEncoder.setInverted(true);

    m_pivotPID = m_pivotMotorLeft.getPIDController();
    m_pivotPID.setP(0.11);
    m_pivotPID.setI(0);
    m_pivotPID.setD(0);
    m_pivotPID.setFF(0);
    m_pivotPID.setFeedbackDevice(m_pivotEncoder);

    // pivotTuner = new PIDTuner(m_pivotPID, ()-> pivotPIDTarget)
    //                 .withName("Shooter Pivot")
    //                 .withValue(m_pivotEncoder::getPosition)
    //                 .withOutput(m_pivotMotor::get);
    // pivotTuner.initializeValues(m_pivotPID);

    SmartDashboard.putNumber("Shooter Tuning Angle", 255);

    m_pivotMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);
    m_pivotMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    m_pivotMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    m_pivotMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); // analog sensor position
    m_pivotMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    m_pivotMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10); // duty cycle position
    m_pivotMotorLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); // duty cycle velocity
  }

  double error = 0;
  boolean isAligned = false;

  @Override
  public void periodic() {
    position = m_pivotEncoder.getPosition();
    error = pivotPIDTarget - position;
    isAligned = Math.abs(error) <= 0.3;

    Logger.recordOutput("Shooter Pivot Aligned", isAligned);
    Logger.recordOutput("Shooter Pivot Position", position);
    Logger.recordOutput("Shooter Pivot Error", error);
    Logger.recordOutput("Motors/Pivot Temperature (C)", m_pivotMotorLeft.getMotorTemperature());
    Logger.recordOutput("Motors/Pivot Current (A)", m_pivotMotorLeft.getOutputCurrent());

    if (pivotTuner != null) pivotTuner.periodic();
  }

  /**
   * Set the shooter position (pivot) to the angle that corresponds to the current distance from the target
   */
  public void setPositionDistanceBased() {
    setPosition(InterpolatorTables.pivotAngleTable().get(RobotContainer.instance.vision.getSpeakerTableKey()));
  }

  public double getCurrentPostion() {
    return position;
  }

  /**
   * Set the shooter position (pivot) to the given angle
   * @param angle The angle to set the pivot to
   */
  public void setPosition(double angle) {
    angle = boundPivot(angle);
    m_pivotPID.setReference(angle, ControlType.kPosition);
    pivotPIDTarget = angle;
    Logger.recordOutput("Shooter Pivot Target", pivotPIDTarget);
  }

  public boolean isAligned() {
    return isAligned;
  }

  public double boundPivot(double target) {
    if(target < pivotMin){
      return pivotMin;
    }
    if(target > pivotMax){
      return pivotMax;
    }
    return target;
  }

  public void stopPivot(){
    m_pivotMotorLeft.stopMotor();
  }

  public Command setPivotSpeedCommand(double speed) {
    return runEnd(
            ()-> {
              m_pivotMotorLeft.set(speed);
            },
            () -> {
              m_pivotMotorLeft.set(0);
            }
    );
  }
}
