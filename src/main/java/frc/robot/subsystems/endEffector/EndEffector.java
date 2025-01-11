package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;

public class EndEffector extends SubsystemBase {

    public static EndEffector create() {
        return RobotBase.isReal() ? 
            new EndEffector(new EndEffectorIOReal()) :
            new EndEffector(new EndEffectorIOIdeal());
    }

    public static EndEffector disable(){
        return new EndEffector(new EndEffectorIOIdeal());
    }


    
    private EndEffectorIO io; 
    private EndEffectorInputs inputs;

    public EndEffector(EndEffectorIO io){
        this.io = io;
        this.inputs = new EndEffectorInputs();
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
    }

    public Command intakeCoral(){
        return runOnce(()->{
            io.setVoltage(EndEffectorConstants.kIntakeVoltage);
        })
        .until(() -> inputs.hasPiece)
        .finallyDo(() -> {
            io.setVoltage(Volts.of(0));
        });
    }

    public Command outtakeCoral(){
        return runOnce(()->{
            io.setVoltage(EndEffectorConstants.kOuttakeVoltage);
        });
    }
}
