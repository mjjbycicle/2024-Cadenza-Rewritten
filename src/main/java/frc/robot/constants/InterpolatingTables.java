package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.shooter.constants.ShooterConstants;

public class InterpolatingTables {
  private static double[][] shots;
  private static InterpolatingDoubleTreeMap angleTable, speedTable;

  private static final double ANGLE_OFFSET = 0;

  public static void initializeTables() {
    angleTable = new InterpolatingDoubleTreeMap();
    speedTable = new InterpolatingDoubleTreeMap();

    shots =
        new double[][] {
          {1.2, 0.5, ShooterConstants.ShooterLowerOffset - 0.095},
          {2.1, 0.5, ShooterConstants.ShooterLowerOffset - 0.048},
          {2.6, 0.5, ShooterConstants.ShooterLowerOffset - 0.0325},
          {3.1, 0.5, ShooterConstants.ShooterLowerOffset - 0.02},
          {4.0, 0.5, ShooterConstants.ShooterLowerOffset - 0.01},
          {4.1, 0.5, ShooterConstants.ShooterLowerOffset - 0.005},
          {4.3, 0.5, ShooterConstants.ShooterLowerOffset}
        };

    for (double[] shot : shots) {
      speedTable.put(shot[0], shot[1]);
      angleTable.put(shot[0], shot[2] + ANGLE_OFFSET);
    }
  }

  public static InterpolatingDoubleTreeMap getSpeedTable() {
    return speedTable;
  }

  public static InterpolatingDoubleTreeMap getAngleTable() {
    return angleTable;
  }
}
