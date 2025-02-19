/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.climb;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbImpl extends Climb {
    private TalonFX motor;
    // private CANcoder absoluteEncoder;

    private Optional<Double> voltageOverride;

    private boolean hasBeenReset;

    private SysIdRoutine sysidRoutine;
    private boolean isRunningSysid;

    protected ClimbImpl() {
        super();
        motor = new TalonFX(Ports.Climb.MOTOR);
        Motors.Climb.MOTOR_CONFIG.configure(motor);
        motor.setPosition(Settings.Climb.OPEN_ANGLE.getRotations());

        // absoluteEncoder = new CANcoder(Ports.Climb.ABSOLUTE_ENCODER);

        // MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
        //     .withMagnetOffset(Constants.Climb.ANGLE_OFFSET.getRotations())
        //     .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

        // absoluteEncoder.getConfigurator().apply(magnetSensorConfigs);

        voltageOverride = Optional.empty();

        hasBeenReset = false;

        isRunningSysid = false;

        sysidRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, 
                Units.Volts.of(6), 
                null), 
            new SysIdRoutine.Mechanism(
                output -> {
                    motor.setVoltage(output.in(Units.Volts));
                    isRunningSysid = true;
                }, 
                state -> SignalLogger.writeString("SysIdElevator_State", state.toString()), 
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

    private Rotation2d getTargetAngle() {
        switch (getState()) {
            case STOW:
                return Settings.Climb.CLOSED_ANGLE;
            case OPEN:
                return Settings.Climb.OPEN_ANGLE;
            case CLIMBING:
                return Settings.Climb.CLIMBED_ANGLE;
            default:
                return Settings.Climb.CLOSED_ANGLE;
        }
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getTargetAngle().getDegrees() - getCurrentAngle().getDegrees()) < Settings.Climb.ANGLE_TOLERANCE.getDegrees();
    }

    @Override
    public void setVoltageOverride(Optional<Double> voltage) {
        this.voltageOverride = voltage;
    }

    @Override
    public void periodic() {
        if (Settings.EnabledSubsystems.CLIMB.get()) {
            if (!isRunningSysid) {
                if (!hasBeenReset) {
                    motor.setVoltage(Settings.Climb.RESET_VOLTAGE);
                    if (Math.abs(motor.getStatorCurrent().getValueAsDouble()) > Settings.Climb.RESET_STALL_CURRENT) {
                        hasBeenReset = true;
                        motor.setPosition(Settings.Climb.OPEN_ANGLE.getRotations());
                    }
                }
                else {
                    if (voltageOverride.isPresent()) {
                        motor.setVoltage(voltageOverride.get());
                    }
                    else {
                        if (!atTargetAngle()) {
                            double voltageMagnitude = getState() == ClimbState.CLIMBING ? Settings.Climb.CLIMB_VOLTAGE : Settings.Climb.DEFAULT_VOLTAGE;
                            motor.setVoltage(getTargetAngle().getDegrees() - getCurrentAngle().getDegrees() > 0
                                ? voltageMagnitude
                                : -voltageMagnitude);
                        }
                        else {
                            motor.setVoltage(0);
                        }
                        // motor.setControl(new PositionVoltage(getTargetAngle().getRotations()));
                    }
                }
            }
        }
        else {
            motor.setVoltage(0);
        }

        SmartDashboard.putNumber("Climb/Current Angle (deg)", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Climb/Target Angle (deg)", getTargetAngle().getDegrees());

        SmartDashboard.putNumber("Climb/Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb/Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
