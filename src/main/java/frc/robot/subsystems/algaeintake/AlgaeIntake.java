/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeintake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  private AlgaeIntakeIO io;
  private AlgaeIntakeInputs inputs;

  public AlgaeIntake create() {
    return RobotBase.isReal()
        ? new AlgaeIntake(new AlgaeIntakeIOSpark())
        : new AlgaeIntake(new AlgaeIntakeIOIdeal());
  }

  public AlgaeIntake(AlgaeIntakeIO io) {
    this.io = io;
    this.inputs = new AlgaeIntakeInputs();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public Command off() {
    return run(
        () -> {
          io.setVoltage(Volts.of(0));
        });
  }

  public Command in() {
    return run(
        () -> {
          io.setVoltage(AlgaeIntakeConstants.kIntakeVolts);
        });
  }

  public Command out() {
    return run(
        () -> {
          io.setVoltage(AlgaeIntakeConstants.kOuttakeVolts);
        });
  }
}
