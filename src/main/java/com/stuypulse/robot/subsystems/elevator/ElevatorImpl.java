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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorImpl extends Elevator {
    private final TalonFX motor;
    private final SmartNumber targetHeight;
    private final DigitalInput bumpSwitchBottom;

    public ElevatorImpl() {
        motor = new TalonFX(Ports.Elevator.MOTOR);
        Motors.Elevator.MOTOR_CONFIG.configure(motor);

        bumpSwitchBottom = new DigitalInput(Ports.Elevator.BOTTOM_SWITCH);

        targetHeight =
                new SmartNumber("Elevator/Target Height", Constants.Elevator.MIN_HEIGHT_METERS);
    }

    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(
                SLMath.clamp(
                        height,
                        Constants.Elevator.MIN_HEIGHT_METERS,
                        Constants.Elevator.MAX_HEIGHT_METERS));
    }

    @Override
    public double getTargetHeight() {
        return targetHeight.getAsDouble();
    }

    @Override
    public double getCurrentHeight() {
        return motor.getPosition().getValueAsDouble()
                * Constants.Elevator.Encoders.DISTANCE_PER_ROTATION;
    }

    @Override
    public boolean atTargetHeight() {
        return Math.abs(getTargetHeight() - getCurrentHeight())
                < Settings.Elevator.HEIGHT_TOLERANCE_METERS;
    }

    @Override
    public boolean atBottom() {
        return !bumpSwitchBottom.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        motor.setControl(
                new MotionMagicVoltage(
                        getTargetHeight()
                                / Constants.Elevator.Encoders.POSITION_CONVERSION_FACTOR));

        if (atBottom()) {
            motor.setPosition(
                    Constants.Elevator.MIN_HEIGHT_METERS
                            / Constants.Elevator.Encoders.POSITION_CONVERSION_FACTOR);
        }

        SmartDashboard.putNumber("Elevator/Current Height (m)", getCurrentHeight());
        SmartDashboard.putNumber("Elevator/Target Height (m)", getTargetHeight());

        SmartDashboard.putNumber("Elevator/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Motor Current", motor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Elevator/Supply Voltage", motor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
    }
}
