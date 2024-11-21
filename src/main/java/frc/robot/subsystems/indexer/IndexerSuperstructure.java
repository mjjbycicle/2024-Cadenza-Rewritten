package frc.robot.subsystems.indexer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.indexer.constants.IndexerConstants;
import frc.robot.subsystems.indexer.constants.IndexerState;
import frc.robot.subsystems.indexer.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.intake.IntakeSubsystem;

import java.util.Objects;

public class IndexerSuperstructure extends SubsystemBase {
    private static IndexerSuperstructure instance;

    public static IndexerSuperstructure getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, IndexerSuperstructure::new);
    }

    private final HopperSubsystem hopper;
    private final IntakeSubsystem intake;
    private IndexerState state;

    private final Mechanism2d intakeMech;
    private final MechanismLigament2d intakePivot;

    private IndexerSuperstructure() {
        hopper = HopperSubsystem.getInstance();
        intake = IntakeSubsystem.getInstance();
        intakeMech = new Mechanism2d(6, 6);
        MechanismRoot2d intakeRoot = intakeMech.getRoot(
                "Intake Root",
                4, 1
        );
        intakePivot = intakeRoot.append(
                new MechanismLigament2d(
                        "Intake Pivot",
                        3,
                        Units.radiansToDegrees(intake.getCurrentAngle())
                )
        );
        setIndexerState(IndexerState.IDLE);
    }

    private void setIndexerState(IndexerState state) {
        this.state = state;
        hopper.setIndexerState(state);
        intake.setIntakeState(state);
    }

    public Command setIndexerStateCommand(IndexerState state) {
        return Commands.runOnce(() -> setIndexerState(state));
    }

    @Override
    public void periodic() {
        if (state == IndexerState.AMP_PREP && !hopper.hasNoteTrigger().debounce(1).getAsBoolean()) {
            setIndexerState(IndexerState.AMP_READY);
        }
        intakePivot.setAngle(Units.radiansToDegrees(intake.getCurrentAngle()));
        SmartDashboard.putData("Intake Arm", intakeMech);
    }

    public Trigger hopperHasNote() {
        return hopper.hasNoteTrigger();
    }

    public static double getPivotPercent(double intakeDist) {
        double lowerOffset = IndexerConstants.INTAKE_PIVOT_EXTENDED_SETPOINT;
        double higherOffset = IndexerConstants.INTAKE_PIVOT_DEFAULT_SETPOINT;
        return (intakeDist - lowerOffset) / (higherOffset - lowerOffset);
    }

    public static double getPivotRotation(double intakeDist) {
        double percent = getPivotPercent(intakeDist);
        return percent / 4;
    }

    public static double getPivotRadians(double intakeDist) {
        return Units.rotationsToRadians(getPivotRotation(intakeDist));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Indexer");
        builder.addBooleanProperty(
                "Has Note",
                this.hopperHasNote(),
                null
        );
        builder.addStringProperty(
                "State",
                () -> state.name(),
                null
        );
    }
}
