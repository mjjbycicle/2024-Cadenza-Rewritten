package frc.robot.constants;

import frc.robot.oi.OI;
import frc.robot.oi.OI.Buttons;

public final class Controls {

  public static final class DriverControls {
    public static final OI.Axes SwerveForwardAxis = OI.Axes.LEFT_STICK_Y;
    public static final OI.Axes SwerveStrafeAxis = OI.Axes.LEFT_STICK_X;
    public static final OI.Axes SwerveRotationAxis = OI.Axes.RIGHT_STICK_X;
    public static final Buttons AimButton = Buttons.LEFT_TRIGGER;

    public static final Buttons ClimberExtendButton = Buttons.LEFT_BUMPER;
    public static final Buttons ClimberRetractButton = Buttons.RIGHT_BUMPER;
    public static final Buttons ClimberSwap1Button = Buttons.POV_LEFT;
    public static final Buttons ClimberSwap2Button = Buttons.POV_RIGHT;

    public static final Buttons AmpAlignButton = Buttons.X_BUTTON;

    public static final Buttons[] ResetNavXButtons = {Buttons.A_BUTTON, Buttons.B_BUTTON};
  }

  public static final class OperatorControls {
    public static final Buttons RunSpeakerShooterButton = Buttons.RIGHT_TRIGGER;
    public static final Buttons RunAmpShooterButton = Buttons.LEFT_BUMPER;
    public static final Buttons ManualShooterButton = Buttons.LEFT_TRIGGER;

    public static final Buttons IntakeButton = Buttons.X_BUTTON;
    public static final Buttons OuttakeButton = Buttons.Y_BUTTON;
    public static final Buttons IntakeExtendButton = Buttons.POV_DOWN;
    public static final Buttons IntakeRetractButton = Buttons.POV_UP;

    public static final Buttons ToggleIR = Buttons.A_BUTTON;

    public static final Buttons LaunchShooterButton = Buttons.RIGHT_BUMPER;

    public static final Buttons FeedShooterButton = Buttons.POV_LEFT;
  }
}
