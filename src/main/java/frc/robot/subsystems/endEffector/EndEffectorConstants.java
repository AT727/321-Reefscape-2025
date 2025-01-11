package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;

public class EndEffectorConstants {
    public static final int kMotorID = 0;
    public static final int kTouchSensorID = 1;

    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final double kNominalVoltage = 12.0;
    public static final boolean kIsInverted = false;
    public static final int kCurrentLimit = 40; 

    public static final Voltage kIntakeVoltage = Volts.of(12);
    public static final Voltage kOuttakeVoltage = Volts.of(-12.0);
}
