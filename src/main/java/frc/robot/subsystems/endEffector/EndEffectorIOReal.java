package frc.robot.subsystems.endEffector;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class EndEffectorIOReal implements EndEffectorIO {

    private SparkMax motor;
    private DigitalInput touchSensor;

    public EndEffectorIOReal(){
        motor = new SparkMax(EndEffectorConstants.kMotorID, MotorType.kBrushless);
        touchSensor = new DigitalInput(EndEffectorConstants.kTouchSensorID);

        configureMotor();

    }

    public void configureMotor(){
        SparkBaseConfig config = new SparkMaxConfig()
            .inverted(EndEffectorConstants.kIsInverted)
            .voltageCompensation(EndEffectorConstants.kNominalVoltage)
            .smartCurrentLimit(EndEffectorConstants.kCurrentLimit)
            .idleMode(EndEffectorConstants.kIdleMode);

        motor.configure(
            config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );
    }

    public void setVoltage(Voltage volts){
        motor.setVoltage(volts);
    }

    public void updateInputs(EndEffectorInputs inputs){
        inputs.voltage = Volts.of(motor.getBusVoltage());
        inputs.hasPiece = touchSensor.get();
    }
}