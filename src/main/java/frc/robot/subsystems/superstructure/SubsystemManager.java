package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.util.Units;
import frc.robot.oi.OI;
import frc.robot.subsystems.drive.*;
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

public class SubsystemManager {
  private static SwerveDriveSubsystem drive;
  private static OI oi;
  private static ShooterSuperStructure shooter;
  private static IntakeSuperstructure intake;

  public static SwerveDriveSubsystem getDrive() {
    if (drive == null) {
      SwerveDriveTrainConstants drivetrain =
          new SwerveDriveTrainConstants().withPigeon2Id(1).withCANbusName("Fred").withTurnKp(5);

      Slot0Configs steerGains = new Slot0Configs();
      Slot0Configs driveGains = new Slot0Configs();

      {
        steerGains.kP = 30;
        steerGains.kD = 0.2;
        driveGains.kP = 1;
      }

      SwerveDriveConstantsCreator m_constantsCreator =
          new SwerveDriveConstantsCreator(
              6.55, // 10:1 ratio for the drive motor
              12.8, // 12.8:1 ratio for the steer motor
              2, // 3 inch radius for the wheels
              17, // Only apply 17 stator amps to prevent slip
              steerGains, // Use the specified steer gains
              driveGains, // Use the specified drive gains
              true // CANcoder not reversed from the steer motor. For WCP Swerve X this should be
              // true.
              );

      SwerveModuleConstants frontRight =
          m_constantsCreator.createModuleConstants(
              1, 2, 9, 0.379, Units.inchesToMeters(27.5 / 2.0), Units.inchesToMeters(-27.5 / 2.0));

      SwerveModuleConstants frontLeft =
          m_constantsCreator.createModuleConstants(
              3, 4, 10, 0.432, Units.inchesToMeters(27.5 / 2.0), Units.inchesToMeters(27.5 / 2.0));
      SwerveModuleConstants backRight =
          m_constantsCreator.createModuleConstants(
              5,
              6,
              11,
              0.734,
              Units.inchesToMeters(-27.5 / 2.0),
              Units.inchesToMeters(-27.5 / 2.0));
      SwerveModuleConstants backLeft =
          m_constantsCreator.createModuleConstants(
              7, 8, 12, 0.96, Units.inchesToMeters(-27.5 / 2.0), Units.inchesToMeters(27.5 / 2.0));

      drive = new SwerveDriveSubsystem(drivetrain, frontLeft, frontRight, backLeft, backRight);
    }
    return drive;
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
              getDrive().getPoseMeters());
    }
    return shooter;
  }

  public static IntakeSuperstructure getIntake() {
    if (intake == null) {
      intake =
          new IntakeSuperstructure(
              new Hopper(new HopperIOSparkMax()),
              new Intake(new IntakeIOSparkMax()),
              new IRSensor());
    }
    return intake;
  }
}
