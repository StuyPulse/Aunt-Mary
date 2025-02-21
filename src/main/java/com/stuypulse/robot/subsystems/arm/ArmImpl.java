/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.ArmDriveFeedForward;
import com.stuypulse.robot.util.ArmElevatorFeedForward;
import com.stuypulse.robot.util.SettableNumber;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

public class ArmImpl extends Arm {

    private TalonFX motor;
    private DutyCycleEncoder absoluteEncoder;

    private Controller controller;

    private Optional<Double> voltageOverride;
    private Rotation2d operatorOffset;
    private SettableNumber kP, kI, kD, kS, kV, kA, kG;

    public ArmImpl() {
        super();
        motor = new TalonFX(Ports.Arm.MOTOR);
        Motors.Arm.MOTOR_CONFIG.configure(motor);
        motor.setPosition(Settings.Arm.MIN_ANGLE.getRotations());

        absoluteEncoder = new DutyCycleEncoder(Ports.Arm.ABSOLUTE_ENCODER);
        absoluteEncoder.setInverted(true);

        MotionProfile motionProfile = new MotionProfile(Settings.Arm.MAX_VEL.getDegrees(), Settings.Arm.MAX_ACCEL.getDegrees());
        motionProfile.reset(Settings.Arm.MIN_ANGLE.getDegrees());

        controller = new MotorFeedforward(kS, kV, kA).position()
            .add(new ArmFeedforward(kG))
            .add(new ArmDriveFeedForward(kG, CommandSwerveDrivetrain.getInstance()::getXAccelGs))
            .add(new ArmElevatorFeedForward(kG, Elevator.getInstance()::getAccelGs))
            .add(new PIDController(kP, kI, kD))
            .setSetpointFilter(motionProfile);

        voltageOverride = Optional.empty();
        operatorOffset = Rotation2d.kZero;

        kP = new SettableNumber(Gains.Arm.Empty.PID.kP);
        kI = new SettableNumber(Gains.Arm.Empty.PID.kI);
        kD = new SettableNumber(Gains.Arm.Empty.PID.kD);
        kS = new SettableNumber(Gains.Arm.Empty.FF.kS);
        kV = new SettableNumber(Gains.Arm.Empty.FF.kV);
        kA = new SettableNumber(Gains.Arm.Empty.FF.kA);
        kG = new SettableNumber(Gains.Arm.Empty.FF.kG);
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
            3, 
            7, 
            "Arm", 
            voltage -> setVoltageOverride(Optional.of(voltage)), 
            () -> getCurrentAngle().getDegrees(), 
            () -> Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble()), 
            () -> motor.getMotorVoltage().getValueAsDouble(), 
            getInstance()
        );
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getCurrentAngle().getDegrees() - getTargetAngle().getDegrees()) < Settings.Arm.ANGLE_TOLERANCE.getDegrees();
    }

    private Rotation2d getTargetAngle() {
       return Rotation2d.fromDegrees(
        SLMath.clamp(getState().getTargetAngle().plus(operatorOffset).getDegrees(), Settings.Arm.MIN_ANGLE.getDegrees(), Settings.Arm.MAX_ANGLE.getDegrees()));
    }

    @Override
    public Rotation2d getCurrentAngle() {
        double encoderAngle = absoluteEncoder.get();
        return Rotation2d.fromRotations(encoderAngle > Settings.Arm.MIN_ANGLE.minus(Rotation2d.fromDegrees(15)).getRotations() 
            ? encoderAngle 
            : encoderAngle + 1);
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public double getVoltageOverride() {
        if (voltageOverride.isPresent()) {
            return voltageOverride.get();
        }
        else {
            return 0;
        }
    }

    @Override
    public void setOperatorOffset(Rotation2d offset) {
        this.operatorOffset = offset;
    }

    @Override
    public Rotation2d getOperatorOffset() {
        return this.operatorOffset;
    }

    private void updateGains() {
        if (getState() == ArmState.HOLD_ALGAE || getState() == ArmState.BARGE) {
            kP.set(Gains.Arm.Algae.PID.kP);
            kI.set(Gains.Arm.Algae.PID.kI);
            kD.set(Gains.Arm.Algae.PID.kD);
            kS.set(Gains.Arm.Algae.FF.kS);
            kV.set(Gains.Arm.Algae.FF.kV);
            kA.set(Gains.Arm.Algae.FF.kA);
            kG.set(Gains.Arm.Algae.FF.kG);
        } else if (Shooter.getInstance().hasCoral()) {
            kP.set(Gains.Arm.Coral.PID.kP);
            kI.set(Gains.Arm.Coral.PID.kI);
            kD.set(Gains.Arm.Coral.PID.kD);
            kS.set(Gains.Arm.Coral.FF.kS);
            kV.set(Gains.Arm.Coral.FF.kV);
            kA.set(Gains.Arm.Coral.FF.kA);
            kG.set(Gains.Arm.Coral.FF.kG);
        } else {
            kP.set(Gains.Arm.Empty.PID.kP);
            kI.set(Gains.Arm.Empty.PID.kI);
            kD.set(Gains.Arm.Empty.PID.kD);
            kS.set(Gains.Arm.Empty.FF.kS);
            kV.set(Gains.Arm.Empty.FF.kV);
            kA.set(Gains.Arm.Empty.FF.kA);
            kG.set(Gains.Arm.Empty.FF.kG);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        
        updateGains();
        
        if (Settings.EnabledSubsystems.ARM.get()) {
            if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            }
            else {
                motor.setVoltage(controller.update(getTargetAngle().getDegrees(), getCurrentAngle().getDegrees()));
            }
        }
        else {
            motor.setVoltage(0);
        }

        SmartDashboard.putBoolean("Arm/Is Voltage Override Present", voltageOverride.isPresent());
        SmartDashboard.putNumber("Arm/Voltage Override", getVoltageOverride());

        SmartDashboard.putNumber("Arm/Setpoint (deg)", controller.getSetpoint());
        SmartDashboard.putNumber("Arm/Angle Error (deg)", controller.getError());
        
        SmartDashboard.putBoolean("Arm/Absolute Encoder is Connected", absoluteEncoder.isConnected());
        SmartDashboard.putNumber("Arm/Absolute Encoder Value raw (deg)", Units.rotationsToDegrees(absoluteEncoder.get()));

        SmartDashboard.putNumber("Arm/Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Arm/Stator Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
