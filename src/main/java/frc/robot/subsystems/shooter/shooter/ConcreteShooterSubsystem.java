package frc.robot.subsystems.shooter.shooter;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.TunableFeedforward;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShooterPIDs;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.lib.Helpers;

public class ConcreteShooterSubsystem extends ShooterSubsystem {
    private final CANSparkFlex shooterMotor;
    private final ProfiledPIDController velocityPID;
    private final TunableFeedforward velocityFF;
    private ShootingMode mode;

    public ConcreteShooterSubsystem() {
        shooterMotor = new CANSparkFlex(
                ShooterConstants.IDs.SHOOTER_SHOOTER_MOTOR,
                CANSparkLowLevel.MotorType.kBrushless
        );
//        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        shooterMotor.setInverted(false);
        shooterMotor.setSmartCurrentLimit(50);
        velocityPID = new ProfiledPIDController(
                ShooterPIDs.SHOOTER_SHOOTER_kP.get(),
                0,0,
                new TrapezoidProfile.Constraints(
                        ShooterPIDs.SHOOTER_SHOOTER_ACCELERATION.get(),
                        ShooterPIDs.SHOOTER_SHOOTER_JERK.get()
                )
        );
        velocityFF = new TunableFeedforward(
                0,
                ShooterPIDs.SHOOTER_SHOOTER_kV.get(),
                0
        );
        mode = ShootingMode.IDLE;
    }

    private void runSpeed() {
        double pidOutput =
                velocityPID.getGoal().position;
//                velocityPID.calculate(getCurrentSpeed())
//                + velocityFF.calculate(getCurrentSpeed());
        SmartDashboard.putNumber("shooter output", pidOutput);
        SmartDashboard.putNumber("target shooter speed", getSpeed(mode));
        shooterMotor.set(pidOutput);
    }

    @Override
    public void setShootingMode(ShootingMode mode) {
        this.mode = mode;
        velocityPID.setGoal(getSpeed(mode));
    }

    @Override
    protected boolean atSpeedSetpoint() {
        return Helpers.withinTolerance(
                getCurrentSpeed(),
                getSpeed(mode),
                ShooterConstants.SHOOTER_SPEED_ERROR
        );
    }

    @Override
    public void periodic() {
        runSpeed();
        updateTunables();
    }

    @Override
    public double getCurrentSpeed() {
        return shooterMotor.getEncoder().getVelocity();
    }

    private void updateTunables() {
        velocityPID.setPID(
                ShooterPIDs.SHOOTER_SHOOTER_kP.get(),
                0, 0
        );
        velocityPID.setConstraints(
                new TrapezoidProfile.Constraints(
                        ShooterPIDs.SHOOTER_SHOOTER_ACCELERATION.get(),
                        ShooterPIDs.SHOOTER_SHOOTER_JERK.get()
                )
        );
        velocityFF.setKv(
                ShooterPIDs.SHOOTER_SHOOTER_kV.get()
        );
    }

    @Override
    public double getTargetSpeed() {
        return getSpeed(mode);
    }
}
