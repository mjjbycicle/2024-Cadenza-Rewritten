package frc.robot.subsystems.intake.intake;

import static frc.robot.constants.RobotInfo.IntakeInfo;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.IDs;
import frc.robot.constants.RobotInfo;
import frc.robot.subsystems.intake.hopper.HopperSubsystem;
import frc.robot.util.Helpers;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax pivotMotorLeft;
    private final CANSparkMax pivotMotorRight;
    private final CANSparkMax intakeMotor;

    private final PIDController pivotPID;
    private final DutyCycleEncoder encoder;
    private final DigitalInput ir;

    private final HopperSubsystem hopper;

    private IntakeInfo.ExtenstionState extenstionState;

    public IntakeSubsystem() {
        pivotMotorLeft = new CANSparkMax(IDs.IntakeIDs.pivotLeft, CANSparkMax.MotorType.kBrushless);
        pivotMotorRight = new CANSparkMax(IDs.IntakeIDs.pivotRight, CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IDs.IntakeIDs.intakeMotor, CANSparkMax.MotorType.kBrushless);
        hopper = new HopperSubsystem();

        pivotPID = IntakeInfo.intakePID.create();

        pivotMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pivotMotorLeft.setInverted(false);
        pivotMotorRight.follow(pivotMotorLeft, true);

        encoder = new DutyCycleEncoder(IDs.IntakeIDs.intakeEncoder);

        ir = new DigitalInput(IDs.IntakeIDs.IR1);

        setExtended(IntakeInfo.ExtenstionState.RETRACTED);
    }

    private void setExtension(IntakeInfo.ExtenstionState extended) {
        pivotPID.setSetpoint(
                switch (extended) {
                    case EXTENDED -> IntakeInfo.intakeExtendedSetpoint;
                    case RETRACTED -> IntakeInfo.intakeRetractedSetpoint;
                });
        extenstionState = extended;
    }

    // Spins intake motor to intake notes
    public void intake() {
        intakeMotor.set(IntakeInfo.intakeSpeed);
    }

    public void outtake() {
        intakeMotor.set(-IntakeInfo.intakeSpeed);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        double pidOutput = -pivotPID.calculate(encoder.getAbsolutePosition());

        if (pidOutput > 0) pidOutput *= IntakeInfo.intakeUpMultiplier;

        SmartDashboard.putNumber("target pivot pos", pivotPID.getSetpoint());
        SmartDashboard.putNumber("current pivot pos", getPosition());
        SmartDashboard.putNumber("pivot pid output", pidOutput);

        pivotMotorLeft.set(pidOutput);

        SmartDashboard.putNumber("intake pivot output", pivotMotorRight.getAppliedOutput());

        if (DriverStation.isDisabled()) {
            setExtended(IntakeInfo.ExtenstionState.RETRACTED);
        }
    }

    private double getPosition() {
        return encoder.getAbsolutePosition();
    }

    public Command setExtended(IntakeInfo.ExtenstionState extended) {
        return new RunCommand(() -> setExtension(extended), this);
    }

    public Command intakeCommand() {
        return new RunCommand(this::intake, this);
    }

    public Command outtakeCommand() {
        return new RunCommand(this::outtake, this);
    }

    public boolean isNoteDetected() {
        return !ir.get();
    }

    public Command feedShooterCommand() {
        return new RunCommand(() -> hopper.run(IntakeInfo.hopperSpeed), this);
    }

    public Trigger isAligned() {
        return new Trigger(this::aligned);
    }

    public boolean aligned() {
        return switch (extenstionState) {
            case EXTENDED -> Helpers.withinTolerance(getPosition(), IntakeInfo.intakeExtendedSetpoint, 10);
            case RETRACTED -> Helpers.withinTolerance(getPosition(), IntakeInfo.intakeRetractedSetpoint, 10);
        };
    }
}
