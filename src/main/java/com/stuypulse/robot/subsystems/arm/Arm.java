package com.stuypulse.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    /*
     * GOODJOB GUYS!! I BELIEVE IN YOU!!
     * FIELDS THAT WE NEED
     * - Kraken Motor
     * - Target angle
     * - Absolute Encoder
     * - PID controller
     * - FF controller
     * - instance
     */

    private static Arm instance;

    private SmartNumber targetAngle;

    private AbsoluteEncoder armEncoder;

    private final ProfiledPIDController pidController;
    private final ArmFeedforward ffController;

    static {
        instance = new Arm();
    }

    public Arm() {
        // super();
        targetAngle = new SmartNumber("Arm/Target Angle", 0);
        armMotor = new TalonFX(0);
         



        pidController = new ProfiledPIDController(
            Settings.Arm.PID.kP,
            Settings.Arm.PID.kI,
            Settings.Arm.PID.kD,
            new Constraints(
                Settings.Arm.MAX_VEL,
                Settings.Arm.MAX_ACCEL
            )            
            
            
            
        );
                    
        
        // pidController = new ProfiledPIDController(//kp, ki, kd settings);
}

_}
