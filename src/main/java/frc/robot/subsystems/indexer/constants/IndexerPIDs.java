package frc.robot.subsystems.indexer.constants;

import frc.lib.TunableNumber;

public class IndexerPIDs {
    public static TunableNumber PIVOT_kP = new TunableNumber("INTAKE_PIVOT_kP");
    public static TunableNumber PIVOT_VELOCITY = new TunableNumber("INTAKE_PIVOT_VELOCITY");
    public static TunableNumber PIVOT_ACCELERATION = new TunableNumber("INTAKE_PIVOT_ACCELERATION");

    static {
        PIVOT_kP.setDefault(2.3);
        PIVOT_VELOCITY.setDefault(2.5);
        PIVOT_ACCELERATION.setDefault(1.5);
    }
}
