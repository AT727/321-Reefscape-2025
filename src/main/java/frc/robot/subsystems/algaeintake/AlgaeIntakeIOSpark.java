/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeintake;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.algaeintake.AlgaeIntake.intakeConfig;

@Logged
public class AlgaeIntakeIOSpark implements AlgaeIntakeIO {

  public static final intakeConfig config = new intakeConfig(0.03, 0, 0, 0);

  private SparkMax motor;
  private DigitalInput touchSensor;

  public AlgaeIntakeIOSpark() {
    motor = new SparkMax(AlgaeIntakeConstants.kIntakeId, MotorType.kBrushless);
    touchSensor = new DigitalInput(AlgaeIntakeConstants.kTouchSensorPort);
    configureMotor();

    configurePivotMotor();
  }

  SparkMax pivotMotor = new SparkMax(0, MotorType.kBrushless);
  SparkAbsoluteEncoder motorController = pivotMotor.getAbsoluteEncoder();

  public void configureMotor() {
    SparkBaseConfig config =
        new SparkMaxConfig()
            .inverted(AlgaeIntakeConstants.kInverted)
            .voltageCompensation(12)
            .smartCurrentLimit(AlgaeIntakeConstants.kCurrentLimit)
            .idleMode(IdleMode.kBrake);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void configurePivotMotor() {
    SparkBaseConfig config =
        new SparkMaxConfig()
            .inverted(AlgaeIntakeConstants.kInverted)
            .voltageCompensation(12)
            .smartCurrentLimit(AlgaeIntakeConstants.kCurrentLimit)
            .idleMode(IdleMode.kBrake);

    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void updateInputs(AlgaeIntakeInputs inputs) {
    inputs.voltage = motor.getBusVoltage();
    inputs.hasAlgae = touchSensor.get();
    inputs.angle = motorController.getPosition();
  }
}
