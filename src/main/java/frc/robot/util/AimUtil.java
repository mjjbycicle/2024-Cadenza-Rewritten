package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.DriverStationUtil;
import frc.robot.constants.Controls;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.InterpolatingTables;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AimUtil {
    public static Translation2d getSpeakerPose() {
        Translation2d speakerPose;
        if (DriverStationUtil.isRed()) {
            speakerPose = FieldConstants.speakerPose_red;
        } else {
            speakerPose = FieldConstants.speakerPose_blue;
        }
        return speakerPose;
    }

    public static Pose2d getManualSpeakerPose() {
        return new Pose2d(
                AimUtil.getSpeakerPose()
                        .minus(DriverStationUtil.isRed() ? new Translation2d(2, 0) : new Translation2d(-2, 0)),
                Rotation2d.fromDegrees(DriverStationUtil.isRed() ? 0 : 180));
    }

    public static Translation2d getAmpPose() {
        Translation2d ampPose;
        if (DriverStationUtil.isRed()) {
            ampPose = FieldConstants.ampPose_red;
        } else {
            ampPose = FieldConstants.ampPose_blue;
        }
        return ampPose;
    }

    public static Rotation2d getSpeakerRotation(double sidewaysSpeed) {
        double aimMultiplier = -3;
        if (DriverStationUtil.isRed()) {
            aimMultiplier *= -1;
        }
        return getSpeakerRotation().minus(Rotation2d.fromDegrees(sidewaysSpeed * aimMultiplier));
    }

    public static Translation2d getPoseVector(Translation2d targetTranslation) {
        Pose2d currBotPose = CommandSwerveDrivetrain.getInstance().getPose();
        Translation2d currBotTranslation = currBotPose.getTranslation();
        return currBotTranslation.minus(targetTranslation);
    }

    public static Rotation2d getPoseRotation(Translation2d targetTranslation) {
        return Rotation2d.fromRadians(
                MathUtil.clamp(
                        Math.atan(
                                getPoseVector(targetTranslation).getY() / getPoseVector(targetTranslation).getX()),
                        -2 * Math.PI,
                        2 * Math.PI));
    }

    public static double getPoseDist(Translation2d targetTranslation) {
        Translation2d targetPose = getPoseVector(targetTranslation);
        return Math.hypot(targetPose.getX(), targetPose.getY());
    }

    public static Rotation2d getPoseOffset(Translation2d targetTranslation) {
        Rotation2d targetRotation = getPoseRotation(targetTranslation);
        Rotation2d heading = CommandSwerveDrivetrain.getInstance().getRotation2d();
        return Rotation2d.fromDegrees(
                MathUtil.clamp(heading.minus(targetRotation).getDegrees(), -360, 360));
    }

    public static Translation2d getSpeakerVector() {
        return getPoseVector(getSpeakerPose());
    }

    public static Rotation2d getSpeakerRotation() {
        return getPoseRotation(getSpeakerPose());
    }

    public static double getSpeakerDist() {
        return getPoseDist(getSpeakerPose());
    }

    public static Rotation2d getSpeakerOffset() {
        return getPoseOffset(getSpeakerPose());
    }

    public static Rotation2d getAmpRotation() {
        return Rotation2d.fromDegrees(90);
    }

    public static Rotation2d getAmpOffset() {
        Rotation2d heading = CommandSwerveDrivetrain.getInstance().getRotation2d();
        return Rotation2d.fromDegrees(
                MathUtil.clamp(getAmpRotation().minus(heading).getDegrees(), -360, 360));
    }

    public static Translation2d getAmpVector() {
        return getPoseVector(getAmpPose());
    }

    public static ShooterSetpoint getShooterSetpoint() {
        double dist = getSpeakerDist();
        double angle = InterpolatingTables.getAngleTable().get(dist);
        double speed = InterpolatingTables.getSpeedTable().get(dist);
        angle -= Controls.DriverControls.SwerveForwardAxis.getAsDouble() * 0.005;
        speed -= Controls.DriverControls.SwerveForwardAxis.getAsDouble() * 0.005;
        return new ShooterSetpoint(speed, angle);
    }
}
