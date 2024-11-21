package frc.robot.subsystems.shooter.constants;

import frc.robot.util.AimUtil;
import frc.robot.util.ShooterSetpoint;

public enum ShootingMode {
    IDLE,
    MANUAL,
    AUTO,
    LAUNCH;

    public ShooterSetpoint getSetPoint() {
        return switch (this) {
            case IDLE -> new ShooterSetpoint(
                        0,
                        ShooterConstants.SHOOTER_MANUAL_ANGLE
                );
            case MANUAL -> new ShooterSetpoint(
                        ShooterConstants.SHOOTER_MANUAL_SPEED,
                        ShooterConstants.SHOOTER_MANUAL_ANGLE
                );
            case AUTO -> AimUtil.getShooterSetpoint();
            case LAUNCH -> new ShooterSetpoint(
                        ShooterConstants.SHOOTER_LAUNCH_SPEED,
                        ShooterConstants.SHOOTER_LAUNCH_ANGLE
                );
        };
    }
}
