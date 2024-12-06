package frc.robot.subsystems.shooter.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.robot.util.AimUtil;

public abstract class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance;

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            if (Robot.isReal()) instance = new ConcreteShooterSubsystem();
            else instance = new SimShooterSubsystem();
        }
        return instance;
    }

    public Trigger atSpeedTrigger() {
        return new Trigger(this::atSpeedSetpoint);
    }

    public abstract void setShootingMode(ShootingMode mode);

    protected abstract boolean atSpeedSetpoint();

    public abstract double getCurrentSpeed();

    public abstract double getTargetSpeed();
}
