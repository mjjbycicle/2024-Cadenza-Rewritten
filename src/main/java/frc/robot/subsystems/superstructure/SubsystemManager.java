package frc.robot.subsystems.superstructure;

import static frc.robot.constants.Constants.currentMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.oi.OI;
import frc.robot.subsystems.NavX.NavX;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIONavX2;
import frc.robot.subsystems.drive.modules.ModuleIO;
import frc.robot.subsystems.drive.modules.ModuleIOSim;
import frc.robot.subsystems.drive.modules.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IRSensor.IRSensor;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.hopper.Hopper;
import frc.robot.subsystems.intake.hopper.HopperIOSparkMax;
import frc.robot.subsystems.intake.intake.Intake;
import frc.robot.subsystems.intake.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.ShooterSuperStructure;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.subsystems.shooter.pivot.PivotIOSparkMax;
import frc.robot.subsystems.shooter.shooter.Shooter;
import frc.robot.subsystems.shooter.shooter.ShooterIOTalonFX;
import frc.robot.util.MovementUtil;

public class SubsystemManager {
    private static NavX navx;
    private static Drive drive;
    private static OI oi;
    private static MovementUtil movementUtil;
    private static ShooterSuperStructure shooter;
    private static IntakeSuperstructure intake;

    public static Drive getDrive() {
        MovementUtil.configure(
                new Translation2d(),
                new Translation2d(),
                new Rotation2d(),
                false,
                false,
                false
        );
        if (drive == null) {
            switch (currentMode) {
                case REAL:
                    drive =
                            new Drive(
                                    new GyroIONavX2(),
                                    new ModuleIOTalonFX(0),
                                    new ModuleIOTalonFX(1),
                                    new ModuleIOTalonFX(2),
                                    new ModuleIOTalonFX(3));
                    break;

                case SIM:
                    drive =
                            new Drive(
                                    new GyroIO() {
                                    },
                                    new ModuleIOSim(),
                                    new ModuleIOSim(),
                                    new ModuleIOSim(),
                                    new ModuleIOSim());
                    break;

                default:
                    drive =
                            new Drive(
                                    new GyroIO() {
                                    },
                                    new ModuleIO() {
                                    },
                                    new ModuleIO() {
                                    },
                                    new ModuleIO() {
                                    },
                                    new ModuleIO() {
                                    });
                    break;
            }
        }
        return drive;
    }

    public static NavX getNavX() {
        if (navx == null) {
            navx = new NavX();
        }
        return navx;
    }

    public static OI getOI() {
        if (oi == null) {
            oi = new OI();
        }
        return oi;
    }

    public static ShooterSuperStructure getShooter() {
        if (shooter == null) {
            shooter =
                    new ShooterSuperStructure(
                            new Shooter(new ShooterIOTalonFX()),
                            new Pivot(new PivotIOSparkMax()),
                            getDrive().getPose());
        }
        return shooter;
    }

    public static IntakeSuperstructure getIntake() {
        if (intake == null) {
            intake =
                    new IntakeSuperstructure(
                            new Hopper(
                                    new HopperIOSparkMax()
                            ),
                            new Intake(
                                    new IntakeIOSparkMax()
                            ),
                            new IRSensor()
                    );
        }
        return intake;
    }
}
