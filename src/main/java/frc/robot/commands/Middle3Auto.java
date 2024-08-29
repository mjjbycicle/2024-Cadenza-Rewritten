package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.RobotInfo;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.intake.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.pivot.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.shooter.ShooterSubsystem;

import static frc.robot.util.AutoHelper.*;

public class Middle3Auto extends SequentialCommandGroup {

  public Middle3Auto(DrivetrainSubsystem drivetrain, VisionSubsystem vision,
                     ShooterSubsystem shooter,
                     ShooterPivotSubsystem shooterPivot,
                     IntakeSubsystem intake,
                     boolean middle3) {
    
    Command middle3Sequence;
    if (middle3) {
      middle3Sequence = Commands.sequence(
        intakeMoveAim("Middle_3", false, 0.5, null),
        shootIfNote()
      ).alongWith(Commands.runOnce(() -> { Robot.TAG_HEIGHT_AIM = true; }));
    } else {
      middle3Sequence = Commands.none();
    }

    addCommands(
      Commands.deadline(
        Commands.sequence(
          Commands.parallel(
            drivetrain.followChoreoPath("FourNote_OTF.1", true, speakerHeading(0.4)),
            Commands.sequence(
              Commands.waitSeconds(0.4),
              shootWhenAligned(0.95 - 0.4),
              Commands.parallel(
                intakeSequence().withTimeout(2.5),
                vision.shootOnTheFly(false)
              )
            )
          ),
          shootIfNote(),
          intakeMoveAim("FourNote_OTF.2", false, 0.5, null),
          shootIfNote(),
          middle3Sequence
        ),
        new RunPivot(shooterPivot),
        new RunShooter(shooter, RobotInfo.ShooterInfo.SHOOTER_AUTON_IDLE_SPEED),
        Commands.run(shooter::setShooterThresholdDistanceBased),
        vision.directTagAiming(false),
        vision.allowVisionOdometry(true),
        vision.shootOnTheFly(true)
      )
    );
  }
}
