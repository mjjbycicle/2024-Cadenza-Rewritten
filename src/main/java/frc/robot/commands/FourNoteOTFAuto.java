package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.Robot;
import frc.robot.constants.RobotInfo;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.intake.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.pivot.ShooterPivotSubsystem;
import frc.robot.subsystems.shooter.shooter.ShooterSubsystem;
import frc.robot.util.ChoreoConductor;

import static frc.robot.util.AutoHelper.*;

public class FourNoteOTFAuto extends SequentialCommandGroup {

  public FourNoteOTFAuto(DrivetrainSubsystem drivetrain, VisionSubsystem vision,
                         ShooterSubsystem shooter,
                         ShooterPivotSubsystem shooterPivot,
                         IntakeSubsystem intake,
                         int... notes) {
    boolean shootOnTheFlyFar = false;
    ChoreoConductor conductor = new ChoreoConductor(true)
                                .toggleVisionOdometry(false)
                                .noteBypass(true)
                                .shootOnTheFly(shootOnTheFlyFar);
    
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

          intakeMoveAim("FourNote_OTF.3", false, 0.5, null),
          shootIfNote(),

          Commands.parallel(
            drivetrain.followChoreoPath("FourNoteConductor", false),
            shootOnTheFlyFar ? Commands.none() : Commands.runOnce(() -> { Robot.TAG_HEIGHT_AIM = true; })
           
          ),
          conductor.scoreNotes(notes),
          drivetrain.followChoreoPath("Amp_After", false)
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
