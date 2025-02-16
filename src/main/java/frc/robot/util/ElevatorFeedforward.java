package frc.robot.util;

import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.StructSerializable;

/** 
 * Rewritten verison fo the WPILib ElevatorFeedforward class.
 * (edu.wpi.first.math.controller.ElevatorFeedforward)
 * I wanted s g v and a to be public variables because I have mental health issues.
 */
public class ElevatorFeedforward implements ProtobufSerializable, StructSerializable {
    public double s;
    public double g;
    public double v;
    public double a;

    public ElevatorFeedforward(double s, double g, double v, double a) {
        this.s = s;
        this.g = g;
        this.v = v;
        this.a = a;
    }
    
    public double calculate(double velocity, double acceleration) {
        return s * Math.signum(velocity) + g + v * velocity + a * acceleration;
    }

    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

    public double calculateWithVelocities(double currentVelocity, double nextVelocity) {
        if (a == 0) {
            return s * Math.signum(nextVelocity) + g + v * nextVelocity;
        } else {
            double A = -v / a;
            double B = 1.0 /a;
            double Ad = Math.exp(A * 0.02);
            double Bd = 1.0 / A * (Ad - 1.0) * B;
            return g + s * Math.signum(currentVelocity)
                   + 1.0 / Bd * (nextVelocity - Ad * currentVelocity);
        }
    }
}
