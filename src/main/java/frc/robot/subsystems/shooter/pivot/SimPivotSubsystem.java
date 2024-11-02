package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShooterPIDs;
import frc.robot.subsystems.shooter.constants.ShootingMode;

public class SimPivotSubsystem extends PivotSubsystem {
    private ShootingMode mode;
    private final ProfiledPIDController pivotPID;
    private final SingleJointedArmSim armSim;

    public SimPivotSubsystem() {
        pivotPID = new ProfiledPIDController(
                ShooterPIDs.SHOOTER_PIVOT_kP.get(),
                ShooterPIDs.SHOOTER_PIVOT_kI.get(),
                ShooterPIDs.SHOOTER_PIVOT_kD.get(),
                new TrapezoidProfile.Constraints(
                        ShooterPIDs.SHOOTER_PIVOT_VELOCITY.get(),
                        ShooterPIDs.SHOOTER_PIVOT_ACCELERATION.get()
                )
        );
        setShootingMode(ShootingMode.IDLE);
        armSim =new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        DCMotor.getNeo550(2),
                        SingleJointedArmSim.estimateMOI(0.3, 10),
                        81.0 * 3 / 2
                ),
                DCMotor.getNEO(2),
                81.0 * 3 / 2,
                0.3,
                ShooterSuperstructure.getShooterRadians(getAngle(ShootingMode.IDLE)),
                ShooterSuperstructure.getShooterRadians(ShooterConstants.ShooterLowerOffset - 0.95),
                true,
                ShooterSuperstructure.getShooterRadians(getAngle(ShootingMode.IDLE))
        );
    }

    @Override
    public void setShootingMode(ShootingMode mode) {
        this.mode = mode;
        pivotPID.setGoal(ShooterSuperstructure.getShooterRadians(getAngle(mode)));
    }

    @Override
    protected boolean atAngleSetpoint() {
        return MathUtil.isNear(
                pivotPID.getGoal().position,
                armSim.getAngleRads(),
                ShooterConstants.SHOOTER_PIVOT_ERROR
        );
    }

    @Override
    public double getCurrentAngle() {
        return armSim.getAngleRads();
    }

    @Override
    public void periodic() {
        updatePIDs();
        reachSetpoint();
        armSim.update(0.020);
        armSim.setState(armSim.getAngleRads(), armSim.getVelocityRadPerSec());
    }

    private void reachSetpoint() {
        var pidOutput =
                pivotPID.calculate(
                        armSim.getAngleRads(),
                        ShooterSuperstructure.getShooterRadians(getAngle(mode))
                );
        armSim.setInputVoltage(pidOutput);
    }

    private void updatePIDs() {
        pivotPID.setPID(
                ShooterPIDs.SHOOTER_PIVOT_kP.get(),
                ShooterPIDs.SHOOTER_PIVOT_kI.get(),
                ShooterPIDs.SHOOTER_PIVOT_kD.get()
        );
        pivotPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ShooterPIDs.SHOOTER_PIVOT_VELOCITY.get(),
                        ShooterPIDs.SHOOTER_PIVOT_ACCELERATION.get()
                )
        );
    }
}
