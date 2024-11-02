package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.OI;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.drive.constants.DriveConstants.CURRENT_MAX_ROBOT_MPS;
import static frc.robot.subsystems.drive.constants.DriveConstants.TELOP_ROTATION_SPEED;

public final class Controls {

  public static final class DriverControls {
    public static final DoubleSupplier SwerveForwardAxis =
        () -> {
          OI oi = OI.getInstance();
          return oi.driverController().getLeftY() * CURRENT_MAX_ROBOT_MPS;
        };
    public static final DoubleSupplier SwerveStrafeAxis =
        () -> {
          OI oi = OI.getInstance();
          return -oi.driverController().getLeftX() * CURRENT_MAX_ROBOT_MPS;
        };
    public static final DoubleSupplier SwerveRotationAxis =
        () -> {
          OI oi = OI.getInstance();
          return -oi.driverController().getRightX() * TELOP_ROTATION_SPEED;
        };
    public static final Trigger AimButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.driverController().leftTrigger();
            });
    public static final Trigger SOTF =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.driverController().rightTrigger();
            });
    public static final Trigger ClimberExtendButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.driverController().getLeftBumper();
            });
    public static final Trigger ClimberRetractButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.driverController().getRightBumper();
            });
    public static final Trigger ClimberSwap1Button =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.driverController().getPOV() == 270;
            });
    public static final Trigger ClimberSwap2Button =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.driverController().getPOV() == 90;
            });

    public static final Trigger AmpAlignButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.driverController().getXButton();
            });

    public static final Trigger
        ResetGyroButton1 =
            new Trigger(
                () -> {
                  OI oi = OI.getInstance();
                  return oi.driverController().getAButton();
                }),
        ResetGyroButton2 =
            new Trigger(
                () -> {
                  OI oi = OI.getInstance();
                  return oi.driverController().getBButton();
                });
  }

  public static final class OperatorControls {
    public static final Trigger RunSpeakerShooterButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.operatorController().rightTrigger();
            });
    public static final Trigger RunAmpShooterButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.operatorController().getLeftBumper();
            });
    public static final Trigger ManualShooterButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.operatorController().leftTrigger();
            });

    public static final Trigger IntakeButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.operatorController().getXButton();
            });
    public static final Trigger OuttakeButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.operatorController().getYButton();
            });
    public static final Trigger IntakeExtendButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.operatorController().getPOV() == 180;
            });
    public static final Trigger IntakeRetractButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.operatorController().getPOV() == 0;
            });
    public static final Trigger ToggleIR =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.operatorController().getAButton();
            });
    public static final Trigger LaunchShooterButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.operatorController().getRightBumper();
            });

    public static final Trigger FeedShooterButton =
        new Trigger(
            () -> {
              OI oi = OI.getInstance();
              return oi.operatorController().getPOV() == 270;
            });
  }
}
