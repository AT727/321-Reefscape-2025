package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ArmConstants{
    public static enum Setpoints{
        CORALSCORE(0),
        STATIONPICKUP(30),
        STOW(20);
        
        double angle;
        
        Setpoints(double angle){
            this.angle = angle;
        }
    }

    //real constants
    public static final int kMotorID = 0;
    public static final IdleMode kIdleMode = IdleMode.kBrake;
    public static final double kNominalVoltage = 12.0;
    public static final boolean kIsInverted = false;
    public static final int kCurrentLimit = 40; 
    
    //sim constants
    public static final double GEARING = 5 * 4 * 5; 
    public static final double MOI = 1.06; 
    public static final double LENGTH_METERS = 0.5588;  
}