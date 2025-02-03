/* (C) Robolancers 2025 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevatorarm.ElevatorArm;

public class ScoringMechanismCommands {

  // moves the entire elevator+arm superstructure to a desired state; this should be the go-to way
  // of moving the superstructure, aside from the default subsystem commands
  public Command goToSetpoint(Elevator elevator, ElevatorArm arm, CoralScorerSetpoint setpoint) {
    return elevator
        .goToHeight(() -> setpoint.getElevatorHeight())
        .alongWith(arm.goToAngle(() -> setpoint.getArmAngle()));
  }

  public enum CoralScorerSetpoint {
    // TODO: determine angles empirically
    NEUTRAL(Inches.of(0), Degrees.of(0)),
    FEED_CORAL(Inches.of(0), Degrees.of(0)),
    L1(Inches.of(0), Degrees.of(0)),
    L2(Inches.of(0), Degrees.of(0)),
    L3(Inches.of(0), Degrees.of(0)),
    L4(Inches.of(0), Degrees.of(0));

    private Distance elevatorHeight; // the height of the elevator to got
    private Angle armAngle; // the angle the arm should go to

    CoralScorerSetpoint(Distance elevatorHeight, Angle armAngle) {
      this.armAngle = armAngle;
      this.elevatorHeight = elevatorHeight;
    }

    public Distance getElevatorHeight() {
      return elevatorHeight;
    }

    public Angle getArmAngle() {
      return armAngle;
    }
  }
}
