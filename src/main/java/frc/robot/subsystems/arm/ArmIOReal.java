package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.Arm.ArmConfig;
import frc.robot.subsystems.endEffector.EndEffectorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

// This is the *real* implementation of an arm (eg. this will allow the code to control a REAL arm with REAL sparkmax's and motors and stuff)
@Logged
public class ArmIOReal implements ArmIO {
    public static final ArmConfig config = new ArmConfig(0.05, 0, 0, 0);

    private SparkMax pivot;
    private SparkAbsoluteEncoder absoluteEncoder; 


    public ArmIOReal() {
        pivot = new SparkMax(ArmConstants.kMotorID, MotorType.kBrushless); 
        absoluteEncoder = pivot.getAbsoluteEncoder();

        configureMotors();
    }

    private void configureMotors() {
        SparkBaseConfig config = 
            new SparkMaxConfig()
            .inverted(ArmConstants.kIsInverted)
            .idleMode(ArmConstants.kIdleMode)
            .smartCurrentLimit(ArmConstants.kCurrentLimit)
            .voltageCompensation(ArmConstants.kNominalVoltage)
            .apply(
                new AbsoluteEncoderConfig()
                .positionConversionFactor(360)
                .velocityConversionFactor(0)
            );

        pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.currentAngle = Rotation2d.fromDegrees(absoluteEncoder.getPosition());
        inputs.velocity = absoluteEncoder.getVelocity(); 
    }

    @Override
    public void setVoltage(Voltage volts) {
        pivot.setVoltage(volts);
    }
}
