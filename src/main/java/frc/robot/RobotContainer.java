// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.superstructure.SubsystemManager.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Controls;
import frc.robot.oi.OI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SubsystemManager;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Superstructure superstructure;
    private final OI oi;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        superstructure = new Superstructure();
        oi = SubsystemManager.getOI();
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        oi.operatorController().getButton(Controls.OperatorControls.IntakeExtendButton).onTrue(superstructure.extendIntakeCommand());
        oi.operatorController().getButton(Controls.OperatorControls.IntakeRetractButton).onTrue(superstructure.retractIntakeCommand());

        oi.driverController().getButton(Controls.DriverControls.AimButton).whileTrue(superstructure.movingSpeakerCommand());
        oi.driverController().getButton(Controls.DriverControls.AimButton).onFalse(superstructure.manualDriveCommand());

        oi.operatorController().getButton(Controls.OperatorControls.IntakeButton).whileTrue(superstructure.intakeCommand());
        oi.operatorController().getButton(Controls.OperatorControls.IntakeButton).onFalse(superstructure.manualDriveCommand());

        oi.operatorController().getButton(Controls.OperatorControls.OuttakeButton).whileTrue(superstructure.outtakeCommand());
        oi.operatorController().getButton(Controls.OperatorControls.OuttakeButton).onFalse(superstructure.manualDriveCommand());

        oi.operatorController().getButton(Controls.OperatorControls.LaunchShooterButton).whileTrue(superstructure.launchSpinupCommand());
        oi.operatorController().getButton(Controls.OperatorControls.LaunchShooterButton).onFalse(superstructure.manualDriveCommand());

        oi.operatorController().getButton(Controls.OperatorControls.FeedShooterButton).whileTrue(superstructure.autoLaunchCommand());
        oi.operatorController().getButton(Controls.OperatorControls.FeedShooterButton).onFalse(superstructure.manualSpeakerCommand());

        oi.operatorController().getButton(Controls.OperatorControls.ManualShooterButton).whileTrue(superstructure.manualSpeakerCommand());
        oi.operatorController().getButton(Controls.OperatorControls.ManualShooterButton).onFalse(superstructure.manualDriveCommand());

        oi.driverController().getButton(Controls.DriverControls.AmpAlignButton).whileTrue(superstructure.autoAmpCommand());
        oi.driverController().getButton(Controls.DriverControls.AmpAlignButton).onFalse(superstructure.manualDriveCommand());

        oi.operatorController().getButton(Controls.OperatorControls.ToggleIR).onTrue(superstructure.toggleIRCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
