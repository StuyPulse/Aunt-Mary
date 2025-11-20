
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterImpl extends Shooter {

    private final TalonFX motor;

    private final DigitalInput beamBreak;
    // private final boolean hasCoral;
    private final BStream hasCoral;

    protected ShooterImpl() {
        super();
        motor = new TalonFX(Ports.Shooter.MOTOR, "can_s3");
        Motors.Shooter.MOTOR_CONFIG.configure(motor);

        beamBreak = new DigitalInput(Ports.Shooter.BEAM_BREAK);

        hasCoral = BStream.create(beamBreak).not()
                    .filtered(new BDebounce.Both(Settings.Shooter.HAS_CORAL_DEBOUNCE));
       // hasCoral = true;
    }

    @Override
    public boolean hasCoral() {
        // return hasCoral;
        return hasCoral.get();
    }

    @Override
    public boolean isAboveCoralCurrentThreshold() {
        return Math.abs(motor.getStatorCurrent().getValueAsDouble()) > Settings.Shooter.CORAL_STATOR_CURRENT_THRESHOLD;
    }

    public void setMotorConfig(NeutralModeValue mode) {
        motor.setNeutralMode(mode);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (Settings.EnabledSubsystems.SHOOTER.get()) {
            motor.set(getState().getSpeed());
        }
        else {
            motor.set(0);
        }

        SmartDashboard.putBoolean("Shooter/Has Coral", hasCoral());
        
        if (Settings.DEBUG_MODE) {
            SmartDashboard.putBoolean("Shooter/ Has Coral Raw", beamBreak.get());
            SmartDashboard.putNumber("Shooter/Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Shooter/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Shooter/Stator Current", motor.getStatorCurrent().getValueAsDouble());
        }
    }
}
