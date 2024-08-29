package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.intake.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.pivot.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.shooter.ShooterSubsystem;

import static frc.robot.util.AutoHelper.*;

public class TwoNoteAutoMid extends SequentialCommandGroup {

  public TwoNoteAutoMid(DrivetrainSubsystem drivetrain,
                        VisionSubsystem vision,
                        ShooterSubsystem shooter,
                        ShooterPivotSubsystem shooterPivot,
                        IntakeSubsystem intake,
                        int... notes) {
    addCommands(
      Commands.deadline(
        Commands.sequence(
          intakeMove("TwoNoteMid.1", true),
          shootIfNote(),
          intakeMove("TwoNoteMid.2"),
          shootIfNote()
        ),
        new ReadyShooter(shooter, shooterPivot),
        vision.directTagAiming(true),
        vision.allowVisionOdometry(false)
      )
    );
  }
}
