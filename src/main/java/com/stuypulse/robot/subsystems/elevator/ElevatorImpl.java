/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.elevator;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorImpl extends Elevator {
    private final TalonFX motor;
    private final DigitalInput bumpSwitchBottom;

    private Optional<Double> voltageOverride;
    private double operatorOffset;

    private SysIdRoutine sysidRoutine;
    private boolean isRunningSysid;

    protected ElevatorImpl() {
        super();
        motor = new TalonFX(Ports.Elevator.MOTOR, Settings.CANIVORE_NAME);
        Motors.Elevator.MOTOR_CONFIG.configure(motor);
        motor.setPosition(Constants.Elevator.MIN_HEIGHT_METERS);

        bumpSwitchBottom = new DigitalInput(Ports.Elevator.BOTTOM_SWITCH);

        voltageOverride = Optional.empty();
        operatorOffset = 0;

        isRunningSysid = false;

        sysidRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, 
                Units.Volts.of(5), 
                null,
                state -> SignalLogger.writeString("SysIdElevator_State", state.toString())), 
            new SysIdRoutine.Mechanism(
                output -> {
                    motor.setVoltage(output.in(Units.Volts));
                    isRunningSysid = true;
                }, 
                null, 
                this));
    }

    @Override
    public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysidRoutine.quasistatic(direction);
    }

    @Override
    public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysidRoutine.dynamic(direction);
    }

    @Override
    public double getCurrentHeight() {
        return motor.getPosition().getValueAsDouble();
    }

    private double getTargetHeight() {
        return getState().getTargetHeight() + operatorOffset;
    }

    @Override
    public boolean atTargetHeight() {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < Settings.Elevator.HEIGHT_TOLERANCE_METERS;
    }

    private boolean atBottom() {
        return !bumpSwitchBottom.get();
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void setOperatorOffset(double offset) {
        this.operatorOffset = offset;
    }

    @Override
    public double getOperatorOffset() {
        return this.operatorOffset;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (atBottom()) {
            motor.setPosition(Constants.Elevator.MIN_HEIGHT_METERS);
        }

        if (Settings.EnabledSubsystems.ELEVATOR.get() && !isRunningSysid) {
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


        SmartDashboard.putNumber("Elevator/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Stator Current", motor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Elevator/Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    }
}
