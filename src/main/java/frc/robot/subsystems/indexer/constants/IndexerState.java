package frc.robot.subsystems.indexer.constants;

public enum IndexerState {
    INTAKING(
            IndexerConstants.INTAKE_PIVOT_EXTENDED_SETPOINT,
            IndexerConstants.INTAKE_SPEED,
            IndexerConstants.HOPPER_SPEED
    ),
    OUTTAKING(
            IndexerConstants.INTAKE_PIVOT_DEFAULT_SETPOINT,
            -IndexerConstants.INTAKE_SPEED,
            -IndexerConstants.HOPPER_SPEED
    ),
    AMP_PREP(
            IndexerConstants.INTAKE_PIVOT_EXTENDED_SETPOINT,
            -IndexerConstants.INTAKE_SPEED,
            -IndexerConstants.HOPPER_SPEED
    ),
    AMP_READY(
            IndexerConstants.INTAKE_PIVOT_DEFAULT_SETPOINT,
            0,
            0
    ),
    AMPING(
            IndexerConstants.INTAKE_PIVOT_AMP_SETPOINT,
            IndexerConstants.AMP_OUTTAKE_SPEED,
            0
    ),
    IDLE(
            IndexerConstants.INTAKE_PIVOT_DEFAULT_SETPOINT,
            0,
            0
    ),
    FEED(
            IndexerConstants.INTAKE_PIVOT_DEFAULT_SETPOINT,
            0,
            IndexerConstants.HOPPER_SPEED
    ),
    EXTENDED(
            IndexerConstants.INTAKE_PIVOT_EXTENDED_SETPOINT,
            0,
            0
    );

    private final double angle, intakeSpeed, hopperSpeed;

    IndexerState(double angle, double intakeSpeed, double hopperSpeed) {
        this.angle = angle;
        this.intakeSpeed = intakeSpeed;
        this.hopperSpeed = hopperSpeed;
    }

    public double getAngle() {
        return angle;
    }

    public double getIntakeSpeed() {
        return intakeSpeed;
    }

    public double getHopperSpeed() {
        return hopperSpeed;
    }
}
