package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.robot.subsystems.shooter.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.shooter.ShooterSubsystem;

import java.util.Objects;

public class ShooterSuperstructure extends SubsystemBase {
    private static ShooterSuperstructure instance;

    public static ShooterSuperstructure getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ShooterSuperstructure::new);
    }

    private final ShooterSubsystem shooter;
    private final PivotSubsystem pivot;

    private final Mechanism2d shooterMech;
    private final MechanismLigament2d shooterArm;

    private ShooterSuperstructure() {
        shooter = ShooterSubsystem.getInstance();
        pivot = PivotSubsystem.getInstance();

        shooterMech = new Mechanism2d(6, 6);
        MechanismRoot2d shooterRoot = shooterMech.getRoot("Shooter Root", 1, 1);
        shooterArm = shooterRoot.append(
                new MechanismLigament2d(
                        "Shooter Arm",
                        4.0,
                        Units.radiansToDegrees(pivot.getCurrentAngle())
                )
        );
    }

    public Command setShootingMode(ShootingMode mode) {
        return runOnce(() -> {
            shooter.setShootingMode(mode);
            pivot.setShootingMode(mode);
        });
    }

    public Trigger atRequestedStateTrigger() {
        return shooter.atSpeedTrigger()
                .and(pivot.atAngleTrigger());
    }

    public static double getShooterPercent(double shooterDist) {
        double lowerOffset = ShooterConstants.ShooterLowerOffset;
        double higherOffset = ShooterConstants.ShooterLowerOffset - 0.095;
        return (shooterDist - lowerOffset) / (higherOffset - lowerOffset);
    }

    public static double getShooterRotation(double shooterDist) {
        double percent = getShooterPercent(shooterDist);
        return percent / 6 + 1 / 24.0;
    }

    public static double getShooterRadians(double shooterDist) {
        return Units.rotationsToRadians(getShooterRotation(shooterDist));
    }

    @Override
    public void periodic() {
        shooterArm.setAngle(Units.radiansToDegrees(pivot.getCurrentAngle()));
        SmartDashboard.putData("Shooter Pivot", shooterMech);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");
        builder.addDoubleProperty(
                "Current Speed",
                shooter::getCurrentSpeed,
                null
        );
        builder.addDoubleProperty(
                "Target Speed",
                shooter::getTargetSpeed,
                null
        );
    }
}
