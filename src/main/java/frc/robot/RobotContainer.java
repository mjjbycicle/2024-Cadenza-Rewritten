// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Controls;
import frc.robot.constants.InterpolatingTables;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.constants.ShootingMode;
import frc.robot.subsystems.shooter.shooter.ShooterSubsystem;

public class RobotContainer {
    private final CommandSwerveDrivetrain swerve;
    private final ShooterSuperstructure shooter;

    public RobotContainer() {
        swerve = TunerConstants.DriveTrain;
        shooter = ShooterSuperstructure.getInstance();
        InterpolatingTables.initializeTables();
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.driveFieldCentricCommand());
        shooter.setDefaultCommand(shooter.setShootingMode(ShootingMode.IDLE));
        Controls.DriverControls.SOTF.whileTrue(
                Commands.parallel(
                        swerve.SOTFCommand(),
                        shooter.setShootingMode(ShootingMode.AUTO)
                )
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void sendSubsystemData() {
        SmartDashboard.putData(swerve);
        SmartDashboard.putData(shooter);
    }
}
