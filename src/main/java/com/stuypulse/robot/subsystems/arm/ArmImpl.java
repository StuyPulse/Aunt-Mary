package com.stuypulse.robot.subsystems.arm;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.AbsoluteEncoder;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmImpl extends Arm {

    /*
     * GOODJOB GUYS!! I BELIEVE IN YOU!!
     * FIELDS THAT WE NEED
     * - Kraken Motor       done
     * - Target angle       done
     * - Absolute Encoder   no
     * - PID controller     
     * - FF controller      
     * - instance           done
     */

     
    private static ArmImpl instance;

    private SmartNumber targetAngle;
    private SmartNumber targetVoltage;


    private TalonFX armMotor;
    //ks, kv, ka, kg

   // private final ProfiledPIDController pidController;
    //private final ArmFeedforward ffController;

    static {
        instance = new ArmImpl();
    }

    public ArmImpl() {
        
        // super();
        private final TalonFXConfiguration config = new TalonFXConfiguration();
        targetAngle = new SmartNumber("Arm/Target Angle", 0.0);
        armMotor = new TalonFX(Ports.Arm.ARM_MOTOR);
         
        Slot0Configs slot0 = new Slot0Configs();

        config.Slot0 = slot0;

        slot0.kP = Settings.Arm.PID.kP;
        slot0.kI = Settings.Arm.PID.kI;
        slot0.kD = Settings.Arm.PID.kD;

        slot0.kS = Settings.Arm.FF.kS;
        slot0.kV = Settings.Arm.FF.kV;
        slot0.kA = Settings.Arm.FF.kA;
        slot0.kG = Settings.Arm.FF.kG;

        
        MotionMagicConfig motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Settings.Arm.MotionMagic.MAX_VEL; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = Settings.Arm.MotionMagic.MAX_ACCEL; // Target acceleration of 160 rps/s (0.5 seconds) 

        // actual gooner code
        
         

        // pidController = new ProfiledPIDController(
        //     Settings.Arm.PID.kP,
        //     Settings.Arm.PID.kI,
        //     Settings.Arm.PID.kD,
        //     new Constraints(
        //         Settings.Arm.MAX_VEL,
        //         Settings.Arm.MAX_ACCEL
        //     )  
        // );      
        
        // pidController.enableContinuousInput(-180,180);
            
            

    //     ffController = new ArmFeedforward(
    //         Settings.Arm.FF.kS,
    //         Settings.Arm.FF.kG,
    //         Settings.Arm.FF.kV,
    //         Settings.Arm.FF.kA); 

    }


    public static ArmImpl getInstance() {
        return instance;
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle.set(targetAngle);
    }

    public double getTargetAngle() {
        return targetAngle.getAsDouble();
    }
                    

    public double getArmAngle() {
        return armMotor.getPosition().getValueAsDouble() * 360;
    }

    // public void setPID(double kP, double kI, double kD) {
    //     config.Slot0.kP = kP;
    //     config.Slot0.kI = kI;
    //     config.Slot0.kD = kD;
        
    // }

    // public void setFF(double kS, double kV, double kA, double kG ){
    //     //delete please
    //     //this is not what you do
    //     config.Slot0.kS = kS;
    //     config.Slot0.kV = kV;
    //     config.Slot0.kA = kA;
    //     config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        


    // }

    public void setVoltage(double voltage) {
        
    }

    

    @Override
    public void periodic() { 
        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(Settings.Arm.);
        
    }
}
    
