package frc.robot.subsystems.indexer.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.indexer.constants.IndexerPIDs;
import frc.robot.subsystems.indexer.constants.IndexerState;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShooterPIDs;

public class SimIntakeSubsystem extends IntakeSubsystem {
    private IndexerState state;
    private final ProfiledPIDController pivotPID;
    private final SingleJointedArmSim armSim;

    public SimIntakeSubsystem() {
        state = IndexerState.IDLE;
        pivotPID = new ProfiledPIDController(
                ShooterPIDs.SHOOTER_PIVOT_kP.get(),
                ShooterPIDs.SHOOTER_PIVOT_kI.get(),
                ShooterPIDs.SHOOTER_PIVOT_kD.get(),
                new TrapezoidProfile.Constraints(
                        ShooterPIDs.SHOOTER_PIVOT_VELOCITY.get(),
                        ShooterPIDs.SHOOTER_PIVOT_ACCELERATION.get()
                )
        );
        armSim =new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        DCMotor.getNeo550(2),
                        SingleJointedArmSim.estimateMOI(0.4, 10),
                        81.0 * 5 / 2
                ),
                DCMotor.getNEO(2),
                81.0 * 5 / 2,
                0.4,
                IndexerSuperstructure.getPivotRadians(state.getAngle()),
                ShooterSuperstructure.getShooterRadians(ShooterConstants.ShooterLowerOffset - 0.95),
                true,
                ShooterSuperstructure.getShooterRadians(state.getAngle())
        );
    }

    @Override
    public void setIntakeState(IndexerState state) {
        this.state = state;
        pivotPID.setGoal(state.getAngle());
    }

    private void reachSetpoint() {
        double pidOutput =
                pivotPID.calculate(
                        armSim.getAngleRads(),
                        IndexerSuperstructure.getPivotRadians(getCurrentAngle())
                );
        armSim.setInputVoltage(pidOutput);
    }

    @Override
    public double getCurrentAngle() {
        return armSim.getAngleRads();
    }

    @Override
    public void periodic() {
        updateTunables();
        reachSetpoint();
    }

    private void updateTunables() {
        pivotPID.setP(IndexerPIDs.PIVOT_kP.get());
        pivotPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        IndexerPIDs.PIVOT_VELOCITY.get(),
                        IndexerPIDs.PIVOT_ACCELERATION.get()
                )
        );
    }
}