package frc.robot.subsystems.indexer.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.indexer.constants.IndexerState;

public abstract class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            if (Robot.isReal()) instance = new ConcreteIntakeSubsystem();
            else instance = new SimIntakeSubsystem();
        }
        return instance;
    }

    public abstract void setIntakeState(IndexerState state);

    public abstract double getCurrentAngle();
}
