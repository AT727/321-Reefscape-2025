package frc.robot.subsystems.arm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.arm.Arm.ArmConfig;

@Logged
public class ArmIOIdeal implements ArmIO {

    public static final ArmConfig config = new ArmConfig(0, 0, 0, 0);

    public ArmIOIdeal() {}

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.velocity = 0; 
        inputs.currentAngle = Rotation2d.fromDegrees(0); 
    }

    @Override
    public void setVoltage(Voltage volts) {

    }
}
