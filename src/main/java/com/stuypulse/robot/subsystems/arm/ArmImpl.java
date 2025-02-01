package com.stuypulse.robot.subsystems.arm;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.AbsoluteEncoder;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmImpl extends Arm {

    /*
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

    private TalonFX armMotor;

    private CANcoder armEncoder;
    static {
        instance = new ArmImpl();
    }

    public ArmImpl() {
        

        // super();
        targetAngle = new SmartNumber("Arm/Target Angle", 0.0);
        armMotor = new TalonFX(Ports.Arm.ARM_MOTOR);
        armEncoder = new CANcoder(Ports.Arm.ARM_ENCODER);

        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();

        slot0.kP = Settings.Arm.PID.kP;
        slot0.kI = Settings.Arm.PID.kI;
        slot0.kD = Settings.Arm.PID.kD;

        slot0.kS = Settings.Arm.FF.kS;
        slot0.kV = Settings.Arm.FF.kV;
        slot0.kA = Settings.Arm.FF.kA;
        slot0.kG = Settings.Arm.FF.kG;

        config.Slot0 = slot0;
        config.Feedback.SensorToMechanismRatio = Settings.Arm.GEAR_RATIO;
        config.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;


        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = Settings.Arm.MotionMagic.MAX_VEL; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = Settings.Arm.MotionMagic.MAX_ACCEL; // Target acceleration of 160 rps/s (0.5 seconds) 

        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0; //set as constant later
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0; //set as constant later


        armMotor.getConfigurator().apply(config);
        armMotor.getConfigurator().apply(motionMagicConfigs);


        // actual gooner code

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
    //     config.Slot0.kS = kS;
    //     config.Slot0.kV = kV;
    //     config.Slot0.kA = kA;
    //     config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        


    // }

    


    

    @Override
    public void periodic() { 

        final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(Settings.Arm.MotionMagic.MAX_VEL);
        armMotor.setControl(m_request);

        
        
    }
}
    
