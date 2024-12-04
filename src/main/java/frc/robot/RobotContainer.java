// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.Controls;
import frc.robot.constants.InterpolatingTables;
import frc.robot.oi.OI;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.indexer.IndexerSuperstructure;
import frc.robot.subsystems.indexer.constants.IndexerState;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.constants.ShootingMode;

public class RobotContainer {
    private final CommandSwerveDrivetrain swerve;
    private final ShooterSuperstructure shooter;
    private final IndexerSuperstructure indexer;

    public RobotContainer() {
        swerve = TunerConstants.DriveTrain;
        shooter = ShooterSuperstructure.getInstance();
        indexer = IndexerSuperstructure.getInstance();
        InterpolatingTables.initializeTables();
        configureBindings();
    }

    private void configureBindings() {
        if (!Constants.sysidTuning) {
            swerve.setDefaultCommand(swerve.driveFieldCentricCommand());
            shooter.setDefaultCommand(shooter.setShootingMode(ShootingMode.IDLE));
            indexer.setDefaultCommand(indexer.setIndexerStateCommand(IndexerState.IDLE));
            Controls.DriverControls.SOTF.whileTrue(
                    Commands.parallel(
                            swerve.SOTFCommand(),
                            shooter.setShootingMode(ShootingMode.AUTO),
                            indexer.setIndexerStateCommand(IndexerState.INTAKING)
                    )
            );
        } else {
            new Trigger(OI.getInstance().driverController()::getAButton).whileTrue(swerve.quasistatic(SysIdRoutine.Direction.kForward));
            new Trigger(OI.getInstance().driverController()::getBButton).whileTrue(swerve.quasistatic(SysIdRoutine.Direction.kReverse));
            new Trigger(OI.getInstance().driverController()::getXButton).whileTrue(swerve.dynamic(SysIdRoutine.Direction.kForward));
            new Trigger(OI.getInstance().driverController()::getYButton).whileTrue(swerve.dynamic(SysIdRoutine.Direction.kReverse));
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void sendSubsystemData() {
        SmartDashboard.putData(swerve);
        SmartDashboard.putData(shooter);
        SmartDashboard.putData(indexer);
    }
}
