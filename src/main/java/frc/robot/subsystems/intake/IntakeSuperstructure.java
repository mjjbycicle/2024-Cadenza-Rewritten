package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IRSensor.IRSensor;
import frc.robot.subsystems.intake.hopper.Hopper;
import frc.robot.subsystems.intake.intake.Intake;

public class IntakeSuperstructure extends SubsystemBase {
    private final Hopper hopper;
    private final Intake intake;
    private final IRSensor irSensor;

    public IntakeSuperstructure(Hopper hopper, Intake intake, IRSensor irSensor) {
        this.hopper = hopper;
        this.intake = intake;
        this.irSensor = irSensor;
    }


}
