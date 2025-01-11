package frc.robot.subsystems.endEffector;

import edu.wpi.first.units.measure.Voltage;

public interface EndEffectorIO {
    default void setVoltage(Voltage volts){};

    default void updateInputs(EndEffectorInputs inputs){};
}
