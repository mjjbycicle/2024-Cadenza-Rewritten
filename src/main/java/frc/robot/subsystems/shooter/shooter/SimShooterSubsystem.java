package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
    private final TalonFX shooterMotor;

    public SimShooterSubsystem() {
        mode = ShootingMode.IDLE;
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        DCMotor.getFalcon500(2),
                        10,
                        1
                ),
                DCMotor.getFalcon500(2),
                1
        );
        shooterMotor = new TalonFX(0);
        shooterMotor.getConfigurator().apply(
                new TalonFXConfiguration()
                        .withMotionMagic(
                                new MotionMagicConfigs()
                                        .withMotionMagicCruiseVelocity(
                                             ShooterPIDs.SHOOTER_SHOOTER_ACCELERATION.get()
                                        )
                                        .withMotionMagicAcceleration(
                                                ShooterPIDs.SHOOTER_SHOOTER_JERK.get()
                                        )
                        )
                        .withSlot0(
                                new Slot0Configs()
                                        .withKP(
                                                ShooterPIDs.SHOOTER_SHOOTER_kP.get()
                                        )
                                        .withKS(
                                                ShooterPIDs.SHOOTER_SHOOTER_kV.get()
                                        )
                        )
        );
    }

    @Override
    public void setShootingMode(ShootingMode mode) {
        this.mode = mode;
    }

    @Override
    protected boolean atSpeedSetpoint() {
        return MathUtil.isNear(getCurrentSpeed(), getSpeed(mode), ShooterConstants.SHOOTER_SPEED_ERROR);
    }

    @Override
    public double getCurrentSpeed() {
        return flywheelSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void periodic() {
        reachSetpoint();
        flywheelSim.update(0.020);
        flywheelSim.setState(flywheelSim.getAngularVelocityRPM());
    }

    private void reachSetpoint() {
        shooterMotor.setControl(new MotionMagicVelocityVoltage(getSpeed(mode)));
        flywheelSim.setInputVoltage(shooterMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Motor Output", shooterMotor.getMotorVoltage().getValueAsDouble());
        shooterMotor.getSimState().setRotorVelocity(flywheelSim.getAngularVelocityRPM() / 60.0);
    }

    @Override
    public double getTargetSpeed() {
        return getSpeed(mode);
    }
}
