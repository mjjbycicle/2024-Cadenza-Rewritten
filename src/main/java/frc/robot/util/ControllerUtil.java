package frc.robot.util;

import frc.robot.constants.Controls;
import frc.robot.oi.OI;
import frc.robot.subsystems.superstructure.SubsystemManager;

public class ControllerUtil {
    public static final OI oi = SubsystemManager.getOI();

    public static boolean getLaunchFeed() {
        return oi.operatorController().getButton(Controls.OperatorControls.FeedShooterButton).getAsBoolean();
    }

    public static boolean getLaunchSpinup() {
        return oi.operatorController().getButton(Controls.OperatorControls.LaunchShooterButton).getAsBoolean();
    }

    public static boolean getSpeakerShot() {
        return oi.operatorController().getButton(Controls.OperatorControls.RunSpeakerShooterButton).getAsBoolean();
    }

    public static boolean getAmpShot() {
        return oi.operatorController().getButton(Controls.OperatorControls.RunAmpShooterButton).getAsBoolean();
    }

    public static boolean getIntake() {
        return oi.operatorController().getButton(Controls.OperatorControls.IntakeButton).getAsBoolean();
    }

    public static boolean getOuttake() {
        return oi.operatorController().getButton(Controls.OperatorControls.OuttakeButton).getAsBoolean();
    }

    public static boolean getExtendIntake() {
        return oi.operatorController().getButton(Controls.OperatorControls.IntakeExtendButton).getAsBoolean();
    }

    public static boolean getRetractIntake() {
        return oi.operatorController().getButton(Controls.OperatorControls.IntakeRetractButton).getAsBoolean();
    }

    public static boolean getManualSpeakerShot() {
        return oi.operatorController().getButton(Controls.OperatorControls.ManualShooterButton).getAsBoolean();
    }

    public static boolean getToggleIR() {
        return oi.operatorController().getButton(Controls.OperatorControls.ToggleIR).getAsBoolean();
    }

    public static boolean idle() {
        return !getSpeakerShot()
                && !getLaunchFeed()
                && !getLaunchSpinup()
                && !getAmpShot();
    }
}
