package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Test extends SubsystemBase {
    CANcoder encoder = new CANcoder(0);
    public Test() {

    }
}
