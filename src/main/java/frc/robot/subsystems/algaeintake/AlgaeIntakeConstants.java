/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeintake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class AlgaeIntakeConstants {
  public static final int kIntakeId = 0;
  public static final boolean kInverted = false;
  public static final int kCurrentLimit = 20;

  public static final Voltage kIntakeVolts = Volts.of(10);
  public static final Voltage kOuttakeVolts = Volts.of(-10);

  public static final int kTouchSensorPort = 0;
}
