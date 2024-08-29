package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.intake.IntakeSubsystem;

public class IntakeCommand extends Command {

    public double intakeSpeed;
    public IntakeSubsystem intakeSub;
    public boolean endOnBeamBreak = false;
    public boolean endOnNoteDetected = false;
    public boolean stopOnEnd = true;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSub = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    public IntakeCommand withEndOnBeamBreak(boolean endOnBeamBreak) {
        this.endOnBeamBreak = endOnBeamBreak;
        return this;
    }

    public IntakeCommand withEndOnNoteDetected(boolean endOnNoteDetected) {
        this.endOnNoteDetected = endOnNoteDetected;
        return this;
    }

    public IntakeCommand withStopOnEnd(boolean stopOnEnd) {
        this.stopOnEnd = stopOnEnd;
        return this;
    }

    public boolean beamBroken() {
        return intakeSub.isNoteDetected();
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        double rumble = 0;
        boolean runOuterIntake = false;
        if (intakeSub.isNoteDetected()) {
            intakeSub.stopIntake();
            rumble = 1;
        } else {
            intakeSub.intake();
            rumble = 0;
        }
        RobotContainer.instance.driverController.getHID().setRumble(RumbleType.kBothRumble, rumble);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.instance.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
        if (stopOnEnd) intakeSub.stopIntake();
    }

    @Override
    public boolean isFinished() {
        if (endOnNoteDetected) return intakeSub.isNoteDetected();

        return false;
    }
}
