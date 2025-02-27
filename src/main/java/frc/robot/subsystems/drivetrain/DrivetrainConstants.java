/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

@Logged
public class DrivetrainConstants {
  public record AutoGains(double kP, double kI, double kD) {}

  public static final AutoGains kTranslationGains =
      RobotBase.isReal()
          ? new AutoGains(5, 0, 0) // real
          : new AutoGains(4, 0, 0.2); // sim

  public static final AutoGains kHeadingGains =
      RobotBase.isReal()
          ? new AutoGains(5, 0, 0) // real
          : new AutoGains(2, 0, 0.1); // sim

  public static final AutoGains tuneTranslationGains = new AutoGains(0, 0, 0); // isn't used
  public static final AutoGains tuneHeadingGains = new AutoGains(6, 0, 0); // for heading controller

  public static final Distance kTrackWidth = Inches.of(29);
  public static final Distance kWheelBase = Inches.of(29);

  public static final double kDriveDeadband = 0.03;
  public static final double kRotationDeadband = 0.03;
  public static final AngularVelocity kMaxAngularVelocity = RadiansPerSecond.of(Math.PI * 6);
  public static final LinearVelocity kMaxLinearVelocity =
      MetersPerSecond.of(4.0); // TunerConstants.kSpeedAt12Volts

  public static final Time kLoopDt = Seconds.of(0.02);

  public static final Distance kAlignmentSetpointTranslationTolerance = Inches.one();
  public static final Angle kAlignmentSetpointRotationTolerance = Degrees.of(2.0);

  public static final DriveTrainSimulationConfig kSimConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              COTS.ofMark4(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getKrakenX60(1),
                  COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                  3)) // L3 Gear ratio
          // Configures the track length and track width (spacing between swerve modules)
          .withTrackLengthTrackWidth(
              DrivetrainConstants.kWheelBase, DrivetrainConstants.kTrackWidth)
          // Configures the bumper size (dimensions of the robot bumper) trackwidth + 6 inches
          .withBumperSize(
              DrivetrainConstants.kWheelBase.plus(Inches.of(6)),
              DrivetrainConstants.kTrackWidth.plus(Inches.of(6)))
          .withRobotMass(Pounds.of(113));
}
