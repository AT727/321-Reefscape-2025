package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;


import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Arm extends SubsystemBase {

// class arm e(subsytem) - instance variables, gotoangle, armconfig, constructor(defines instance variables and pid controller and armfeedforward), desired position variable, method to create arm (real, sim, or ideal)
// class arminputs - velocity and angle
// interface armio - default updateinputs, setpower
// class armioreal i(armio) - motors, armconfig, get encoder, updateinputs, setpower
// class armiosim i(armio) - motorsim, armconfig, updateinputs, setpower

    // configuration data that are dependent on each arm implementation (eg. a simulated arm might have different PID values from a real arm)
    public record ArmConfig(double kP, double kI, double kD, double kG) {}    

    // Arm IO (Input-Output), handles input-output for any type of arm we want to control
    private ArmIO io; 
    private ArmInputs inputs; 

    private PIDController armController;
    private ArmFeedforward armFeedforward; 

    private final ArmVisualizer desiredVisualizer = new ArmVisualizer(Color.kRed); 
    private final ArmVisualizer measuredVisualizer = new ArmVisualizer(Color.kGreen); 

    private Rotation2d desiredPosition = new Rotation2d(); 

    public static Arm create() {
        return RobotBase.isReal() ? 
            new Arm(new ArmIOReal(), ArmIOReal.config) : 
            new Arm(new ArmIOSim(), ArmIOSim.config); 
    }

    
    public static Arm disable() {
        return new Arm(new ArmIOIdeal(), ArmIOIdeal.config); 
    }


    public Arm(ArmIO io, ArmConfig config) {
        this.io = io; 
        this.inputs = new ArmInputs();
        
        this.armController = new PIDController(config.kP, config.kI, config.kD); 
        this.armFeedforward = new ArmFeedforward(0, config.kG, 0); 
        
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        measuredVisualizer.update(inputs.currentAngle);
        desiredVisualizer.update(desiredPosition);
    }

    // TODO: I'm leaning towards not using the Trigger structure since we can wrap around it anytime we want? idk I prefer using raw values instead. 4/10 leaning towards not using Triggers. 
    public Command goToAngle(double desiredAngle){
        return run(()->{
        io.setVoltage(Volts.of(armController.calculate(inputs.currentAngle.getDegrees(), desiredAngle)));
        });
    }

    public Command goToCoralScore(){
        return goToAngle(ArmConstants.Setpoints.CORALSCORE.angle);
    }

    public Command goToStationPickup(){
        return goToAngle(ArmConstants.Setpoints.STATIONPICKUP.angle);
    }

    public Command goToGroundPickup(){
        return goToAngle(ArmConstants.Setpoints.STOW.angle);

    }

    public Command tune() {
        // TODO: redo this with the implementation-independent config
        // LoggedDashboardNumber kP = new LoggedDashboardNumber("Arm/kP", io.getConfig().kP()); 
        // LoggedDashboardNumber kI = new LoggedDashboardNumber("Arm/kI", io.getConfig().kI()); 
        // LoggedDashboardNumber kD = new LoggedDashboardNumber("Arm/kD", io.getConfig().kD());
        // LoggedDashboardNumber kG = new LoggedDashboardNumber("Arm/kG", io.getConfig().kG());
        // TODO: is this a "full test"?
        // return runOnce(() -> {
        //     armController.setPID(kP.get(), kI.get(), kD.get());
        //     armFeedforward = new ArmFeedforward(0, kG.get(), 0); 
        // })
        // .andThen(goToAngle(20).until(this::atPosition))
        // .andThen(goToAngle(80).until(this::atPosition))
        // .andThen(goToAngle(40).until(this::atPosition)); 
        return Commands.none(); 
    }

}
