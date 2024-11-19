// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.constants.TunerConstants;

public class RobotContainer {
  private final CommandSwerveDrivetrain swerve;

  public RobotContainer() {
    swerve = TunerConstants.DriveTrain;
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.driveFieldCentricCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void sendSubsystemData() {
    SmartDashboard.putData(swerve);
  }
}
