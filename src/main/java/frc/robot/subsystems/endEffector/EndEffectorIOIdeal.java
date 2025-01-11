package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.Volts;

public class EndEffectorIOIdeal implements EndEffectorIO {

    public EndEffectorIOIdeal(){}

    public void updateInputs(EndEffectorInputs inputs){
        inputs.voltage = Volts.of(0);
        inputs.hasPiece = false;
    }
    
    public void setVoltage(double voltage){}
}