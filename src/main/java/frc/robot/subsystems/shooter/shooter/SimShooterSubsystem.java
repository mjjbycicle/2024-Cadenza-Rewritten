package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShooterPIDs;
import frc.robot.subsystems.shooter.constants.ShootingMode;

public class SimShooterSubsystem extends ShooterSubsystem {
    private ShootingMode mode;
    private final FlywheelSim flywheelSim;
    private final ProfiledPIDController velocityPID;
    private final SimpleMotorFeedforward velocityFF;

    public SimShooterSubsystem() {
        mode = ShootingMode.IDLE;
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getNeoVortex(1),
                        1,
                        1
                ),
                DCMotor.getNeoVortex(1),
                1
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
    }

    @Override
    public void setShootingMode(ShootingMode mode) {
        this.mode = mode;
    }

    @Override
    protected boolean atSpeedSetpoint() {
        return MathUtil.isNear(getCurrentSpeed(),
                getTargetSpeed(),
                ShooterConstants.SHOOTER_SPEED_ERROR
        );
    }

    @Override
    public double getCurrentSpeed() {
        return flywheelSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void periodic() {
        reachSetpoint();
        flywheelSim.update(0.020);
    }

    private void reachSetpoint() {
        velocityPID.setGoal(getTargetSpeed());
        double output = velocityPID.calculate(getCurrentSpeed())
                + velocityFF.calculate(getCurrentSpeed());
        flywheelSim.setInputVoltage(output);
        SmartDashboard.putNumber("Motor Output", output);
    }

    @Override
    public double getTargetSpeed() {
        return mode.getSetPoint().speed();
    }
}
