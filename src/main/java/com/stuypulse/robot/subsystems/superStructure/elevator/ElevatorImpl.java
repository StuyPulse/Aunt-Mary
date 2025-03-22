/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.superStructure.elevator;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorImpl extends Elevator {
    private final TalonFX motor;

    private Optional<Double> voltageOverride;

    private double velLimit;
    private double accelLimit;

    protected ElevatorImpl() {
        super();
        motor = new TalonFX(Ports.Elevator.MOTOR, Settings.CANIVORE_NAME);
        motor.setPosition(Constants.Elevator.MIN_HEIGHT_METERS);

        velLimit = Settings.Elevator.Constraints.MAX_VELOCITY_METERS_PER_SECOND_TELEOP;
        accelLimit = Settings.Elevator.Constraints.MAX_VELOCITY_METERS_PER_SECOND_AUTON;
        Motors.Elevator.MOTOR_CONFIG.withMotionProfile(velLimit, accelLimit);

        Motors.Elevator.MOTOR_CONFIG.configure(motor);

        voltageOverride = Optional.empty();
    }

    @Override
    public SysIdRoutine getSysIdRoutine() {
        return SysId.getRoutine(
            2, 
            7, 
            "Elevator", 
            voltage -> setVoltageOverride(Optional.of(voltage)), 
            () -> getCurrentHeight(), 
            () -> motor.getVelocity().getValueAsDouble(), 
            () -> motor.getMotorVoltage().getValueAsDouble(), 
            getInstance()
        );
    }

    @Override
    public double getCurrentHeight() {
        return motor.getPosition().getValueAsDouble();
    }

    private double getTargetHeight() {
        return SLMath.clamp(getState().getTargetHeight(), Constants.Elevator.MIN_HEIGHT_METERS, Constants.Elevator.MAX_HEIGHT_METERS);
    }

    private boolean isWithinTolerance(double toleranceMeters) {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < toleranceMeters;
    }

    @Override
    public boolean atTargetHeight() {
        return isWithinTolerance(Settings.Elevator.HEIGHT_TOLERANCE_METERS);
    }

    @Override
    public boolean atCanSkipClearanceHeight() {
        return isWithinTolerance(Settings.Elevator.HEIGHT_TOLERANCE_TO_SKIP_CLEARANCE);
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void setMotionProfileConstraints(double velLimitMetersPerSecond, double accelLimitMetersPerSecondSquared) {
        this.velLimit = velLimitMetersPerSecond;
        this.accelLimit = accelLimitMetersPerSecondSquared;
        Motors.Elevator.MOTOR_CONFIG.withMotionProfile(velLimitMetersPerSecond, accelLimitMetersPerSecondSquared);
        Motors.Elevator.MOTOR_CONFIG.configure(motor);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (Settings.EnabledSubsystems.ELEVATOR.get()) {
            if (voltageOverride.isPresent()) {
                motor.setVoltage(voltageOverride.get());
            }
            else {
                motor.setControl(new MotionMagicVoltage(getTargetHeight()));
            }
        }
        else {
            motor.setVoltage(0);
        }

        if (Settings.DEBUG_MODE) {
            SmartDashboard.putNumber("Elevator/Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Elevator/Stator Current", motor.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Elevator/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    
            SmartDashboard.putNumber("Elevator/Constraints/Current Max vel (m per s)", velLimit);
            SmartDashboard.putNumber("Elevator/Constraints/Current Max accel (m per s per s)", accelLimit);
        }
    }
}
