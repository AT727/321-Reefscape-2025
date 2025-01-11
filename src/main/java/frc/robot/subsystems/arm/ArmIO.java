package frc.robot.subsystems.arm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

// IO (input-output) layer for interfacing with *some* version of the arm (this could be a real arm, or a simulated arm, or even no arm!)
// this includes both getting data (inputs) from the arm sensors (like angle), as well as applying outputs to the arm
@Logged
public interface ArmIO {
    // updates the ArmInputs object with the latest data from sensors (based on the implementation of the arm)
    default void updateInputs(ArmInputs inputs) {}

    // sets the voltage output to the arm
    default void setVoltage(Voltage volts) {}
}
