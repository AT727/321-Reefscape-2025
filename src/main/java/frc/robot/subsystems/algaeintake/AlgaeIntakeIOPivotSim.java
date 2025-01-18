/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeintake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.algaeintake.AlgaeIntake.intakeConfig;

@Logged
public class AlgaeIntakeIOPivotSim implements AlgaeIntakeIO {

  public static final intakeConfig config = new intakeConfig(0.03, 0, 0, 0);

  private SingleJointedArmSim pivot;

  public AlgaeIntakeIOPivotSim() {
    pivot =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            AlgaeIntakeConstants.GEARING,
            AlgaeIntakeConstants.MOI,
            AlgaeIntakeConstants.LENGTH_METERS,
            -90,
            Math.toRadians(90),
            true,
            0);
  }

  @Override
  public void updateInputs(AlgaeIntakeInputs inputs) {
    pivot.update(0.02);
    inputs.angle = pivot.getAngleRads();
    inputs.velocity = pivot.getVelocityRadPerSec();
  }

  @Override
  public void setVoltage(Voltage volts) {
    pivot.setInputVoltage(volts.in(Volts));
  }
}
