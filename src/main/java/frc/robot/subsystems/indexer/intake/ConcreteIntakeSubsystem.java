package frc.robot.subsystems.indexer.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.indexer.constants.IndexerConstants;
import frc.robot.subsystems.indexer.constants.IndexerState;

public class ConcreteIntakeSubsystem extends IntakeSubsystem {
    private final CANSparkMax pivotLeft, pivotRight, intake;
    private final ProfiledPIDController pivotPID;
    private final double startAngle;

    private IndexerState state;

    ConcreteIntakeSubsystem() {
        pivotLeft = new CANSparkMax(IndexerConstants.IDs.INTAKE_PIVOT_MOTOR_LEFT,
                CANSparkMax.MotorType.kBrushless);
        pivotRight = new CANSparkMax(IndexerConstants.IDs.INTAKE_PIVOT_MOTOR_RIGHT,
                CANSparkMax.MotorType.kBrushless);
        pivotLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pivotRight.follow(pivotLeft, true);

        intake = new CANSparkMax(IndexerConstants.IDs.INTAKE_MOTOR,
                CANSparkMax.MotorType.kBrushless);
        intake.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pivotPID = new ProfiledPIDController(
                IndexerConstants.IntakeConstants.INTAKE_PIVOT_kp,
                IndexerConstants.IntakeConstants.INTAKE_PIVOT_ki,
                IndexerConstants.IntakeConstants.INTAKE_PIVOT_kd,
                new TrapezoidProfile.Constraints(
                        IndexerConstants.IntakeConstants.INTAKE_PIVOT_VELOCITY,
                        IndexerConstants.IntakeConstants.INTAKE_PIVOT_ACCELERATION
                )
        );

        DutyCycleEncoder encoder = new DutyCycleEncoder(IndexerConstants.IDs.INTAKE_ENCODER_DIO_PORT);
        startAngle = (encoder.getAbsolutePosition() + 1) % 1;

        state = IndexerState.IDLE;
    }

    @Override
    public void setIntakeState(IndexerState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        intake.set(state.getIntakeSpeed());
        pivotPID.setGoal(state.getAngle());
//        pivotLeft.set(pivotPID.calculate(getCurrentAngle()));
    }

    @Override
    protected double getCurrentAngle() {
        return startAngle + pivotLeft.getEncoder().getPosition() / 81.0;
    }
}
