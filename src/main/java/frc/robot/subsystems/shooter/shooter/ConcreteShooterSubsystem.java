package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShooterPIDs;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.lib.Helpers;

public class ConcreteShooterSubsystem extends ShooterSubsystem {
    private final CANSparkFlex shooter;
    private final ProfiledPIDController velocityPID;
    private final SimpleMotorFeedforward velocityFF;
    private ShootingMode mode;

    public ConcreteShooterSubsystem() {
        shooter = new CANSparkFlex(
                ShooterConstants.IDs.SHOOTER_VORTEX,
                CANSparkLowLevel.MotorType.kBrushless
        );
        velocityPID = new ProfiledPIDController(
                ShooterPIDs.SHOOTER_SHOOTER_kP.get(),
                0, 0,
                new TrapezoidProfile.Constraints(
                        ShooterPIDs.SHOOTER_SHOOTER_ACCELERATION.get(),
                        ShooterPIDs.SHOOTER_SHOOTER_JERK.get()
                )
        );
        velocityFF = new SimpleMotorFeedforward(
                0, ShooterPIDs.SHOOTER_SHOOTER_kV.get()
        );
        mode = ShootingMode.IDLE;
    }

    private void runSpeed() {
        double pidOutput = velocityPID.calculate(getCurrentSpeed())
                + velocityFF.calculate(getCurrentSpeed());
        shooter.setVoltage(pidOutput);
    }

    @Override
    public void setShootingMode(ShootingMode mode) {
        this.mode = mode;
    }

    @Override
    protected boolean atSpeedSetpoint() {
        return Helpers.withinTolerance(
                getCurrentSpeed(),
                getTargetSpeed(),
                ShooterConstants.SHOOTER_SPEED_ERROR
        );
    }

    @Override
    public void periodic() {
        updateTunables();
        runSpeed();
    }

    @Override
    public double getCurrentSpeed() {
        return shooter.getEncoder().getVelocity();
    }

    private void updateTunables() {
        velocityPID.setP(ShooterPIDs.SHOOTER_SHOOTER_kP.get());
    }

    @Override
    public double getTargetSpeed() {
        return mode.getSetPoint().speed();
    }
}
