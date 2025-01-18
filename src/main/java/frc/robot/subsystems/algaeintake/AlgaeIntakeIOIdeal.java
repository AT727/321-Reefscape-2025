/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeintake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.algaeintake.AlgaeIntake.intakeConfig;

@Logged
public class AlgaeIntakeIOIdeal implements AlgaeIntakeIO {
  public static final intakeConfig config = new intakeConfig(0, 0, 0, 0);

  public AlgaeIntakeIOIdeal() {}

  @Override
  public void setVoltage(Voltage volts) {}

  @Override
  public void setPivotVoltage(Voltage volts) {}

  @Override
  public void updateInputs(AlgaeIntakeInputs inputs) {}
}
