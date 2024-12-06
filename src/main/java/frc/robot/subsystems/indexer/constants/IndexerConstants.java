package frc.robot.subsystems.indexer.constants;

public class IndexerConstants {
    public static final double HOPPER_SPEED = 0.6;

    public static final double INTAKE_SPEED = -0.95;
    public static final double AMP_OUTTAKE_SPEED = 0.6;
    public static final double INTAKE_PIVOT_DEFAULT_SETPOINT = 0.154;
    public static final double INTAKE_PIVOT_EXTENDED_SETPOINT = 0.83;
    private static final double INTAKE_PIVOT_DIFF = INTAKE_PIVOT_EXTENDED_SETPOINT - INTAKE_PIVOT_DEFAULT_SETPOINT;
    public static final double INTAKE_PIVOT_AMP_SETPOINT = INTAKE_PIVOT_DEFAULT_SETPOINT + INTAKE_PIVOT_DIFF * 0.1;

    public class IDs {
        public static final int HOPPER_MOTOR = 18;
        public static final int IR_SENSOR_1_DIO_PORT = 3;
        public static final int IR_SENSOR_2_DIO_PORT = 6;
        public static final int INTAKE_PIVOT_MOTOR_LEFT = 22;
        public static final int INTAKE_PIVOT_MOTOR_RIGHT = 23;
        public static final int INTAKE_MOTOR = 24;
        public static final int INTAKE_ENCODER_DIO_PORT = 1;
    }
}
