//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.lib;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class TunableFeedforward {
    public double ks;
    public double kv;
    public double ka;

    public TunableFeedforward(double ks, double kv, double ka) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
        if (kv < 0.0) {
            throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
        } else if (ka < 0.0) {
            throw new IllegalArgumentException("ka must be a non-negative number, got " + ka + "!");
        }
    }

    public double calculate(double velocity, double acceleration) {
        return this.ks * Math.signum(velocity) + this.kv * velocity + this.ka * acceleration;
    }

    public double calculate(double currentVelocity, double nextVelocity, double dtSeconds) {
        LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(this.kv, this.ka);
        LinearPlantInversionFeedforward<N1, N1, N1> feedforward = new LinearPlantInversionFeedforward(plant, dtSeconds);
        Matrix<N1, N1> r = MatBuilder.fill(Nat.N1(), Nat.N1(), new double[]{currentVelocity});
        Matrix<N1, N1> nextR = MatBuilder.fill(Nat.N1(), Nat.N1(), new double[]{nextVelocity});
        return this.ks * Math.signum(currentVelocity) + feedforward.calculate(r, nextR).get(0, 0);
    }

    public double calculate(double velocity) {
        return this.calculate(velocity, 0.0);
    }

    public double maxAchievableVelocity(double maxVoltage, double acceleration) {
        return (maxVoltage - this.ks - acceleration * this.ka) / this.kv;
    }

    public double minAchievableVelocity(double maxVoltage, double acceleration) {
        return (-maxVoltage + this.ks - acceleration * this.ka) / this.kv;
    }

    public double maxAchievableAcceleration(double maxVoltage, double velocity) {
        return (maxVoltage - this.ks * Math.signum(velocity) - velocity * this.kv) / this.ka;
    }

    public double minAchievableAcceleration(double maxVoltage, double velocity) {
        return this.maxAchievableAcceleration(-maxVoltage, velocity);
    }

    public void setKs(double ks) {
        this.ks = ks;
    }

    public void setKv(double kv) {
        this.kv = kv;
    }

    public void setKa(double ka) {
        this.ka = ka;
    }
}
