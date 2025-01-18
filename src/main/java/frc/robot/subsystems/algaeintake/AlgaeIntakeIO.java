/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeintake;

import edu.wpi.first.units.measure.Voltage;

public interface AlgaeIntakeIO {
  default void setVoltage(Voltage volts) {}
  ;

  default void updateInputs(AlgaeIntakeInputs inputs) {}
  ;
}
