/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeintake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
  public record intakeConfig(double kP, double kI, double kD, double kG) {}

  private AlgaeIntakeIO io;
  private AlgaeIntakeInputs inputs;

  private PIDController intakeController;
  private ArmFeedforward pivotFeedFoward;

  public AlgaeIntake create() {
    return RobotBase.isReal()
        ? new AlgaeIntake(new AlgaeIntakeIOSpark(), AlgaeIntakeIOSpark.config)
        : new AlgaeIntake(new AlgaeIntakeIOPivotSim(), AlgaeIntakeIOPivotSim.config);
  }

  public AlgaeIntake disable() {
    return new AlgaeIntake(new AlgaeIntakeIOIdeal(), AlgaeIntakeIOIdeal.config);
  }

  public AlgaeIntake(AlgaeIntakeIO io, intakeConfig config) {
    this.io = io;
    this.inputs = new AlgaeIntakeInputs();

    intakeController = new PIDController(config.kP, config.kI, config.kD);
    ArmFeedforward pivotFeedForward = new ArmFeedforward(0, config.kG, 0);
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

  public Command goToAngle(double desiredAngle) {
    return run(
        () -> {
          io.setPivotVoltage(Volts.of(intakeController.calculate(inputs.angle, desiredAngle)));
        });
  }

  public void L4() {
    run(
        () -> {
          goToAngle(AlgaeIntakeConstants.kL4);
        });
  }

  public void L2() {
    run(
        () -> {
          goToAngle(AlgaeIntakeConstants.kL2);
        });
  }

  public void L1() {
    run(
        () -> {
          goToAngle(AlgaeIntakeConstants.kL1);
        });
  }

  public void travel() {
    run(
        () -> {
          goToAngle(AlgaeIntakeConstants.kL4);
        });
  }
}
