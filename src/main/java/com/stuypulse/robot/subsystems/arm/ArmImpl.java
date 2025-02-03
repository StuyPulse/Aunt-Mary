package com.stuypulse.robot.subsystems.arm;


import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {

    private Rotation2d targetAngle;

    private TalonFX armMotor;

    private CANcoder armEncoder;
    

    public ArmImpl() {
        
        targetAngle = Rotation2d.fromDegrees(0.0);
        armMotor = new TalonFX(Ports.Arm.ARM_MOTOR);
        armEncoder = new CANcoder(Ports.Arm.ARM_ENCODER);

        TalonFXConfiguration config = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();

        slot0.kP = Settings.Arm.PID.kP.getAsDouble();
        slot0.kI = Settings.Arm.PID.kI.getAsDouble();
        slot0.kD = Settings.Arm.PID.kD.getAsDouble();

        slot0.kS = Settings.Arm.FF.kS.getAsDouble();
        slot0.kV = Settings.Arm.FF.kV.getAsDouble();
        slot0.kA = Settings.Arm.FF.kA.getAsDouble();
        slot0.kG = Settings.Arm.FF.kG.getAsDouble();
        slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        config.Slot0 = slot0;
        config.Feedback.SensorToMechanismRatio = Settings.Arm.GEAR_RATIO;
        // config.Feedback.RotorToSensorRatio = 0.0;
        config.Feedback.FeedbackRemoteSensorID = armEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;

        motionMagicConfigs.MotionMagicCruiseVelocity = Settings.Arm.MotionMagic.MAX_VEL; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = Settings.Arm.MotionMagic.MAX_ACCEL; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = Settings.Arm.MotionMagic.JERK; 

        MagnetSensorConfigs magnet_config = new MagnetSensorConfigs();
        magnet_config.MagnetOffset = Settings.Arm.ENCODER_OFFSET;

        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Settings.Arm.PID_RAMPING.getAsDouble(); 
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.Arm.FF_RAMPING.getAsDouble(); 
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = Settings.Arm.PID_RAMPING.getAsDouble();
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = Settings.Arm.FF_RAMPING.getAsDouble();
        

        armMotor.getConfigurator().apply(config);
        armMotor.getConfigurator().apply(motionMagicConfigs);
        armEncoder.getConfigurator().apply(magnet_config);

    }

    public void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }
                    

    public Rotation2d getArmAngle() {
        return Rotation2d.fromRotations(armMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() { 

        MotionMagicVoltage armOutput = new MotionMagicVoltage(getTargetAngle().getRotations());
        // armMotor.setControl(new PositionVoltage(getTargetAngle().getRotations()));
        armMotor.setControl(armOutput);

        SmartDashboard.putNumber("Arm/targetAngle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("Arm/currentAngle",getArmAngle().getDegrees());
        
    }
}
    
