/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.funnel;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FunnelImpl extends Funnel {

    private final TalonFX motor;
    private final DigitalInput irSensor;

    private final BStream hasCoral;
    private final BStream shouldReverse;

    protected FunnelImpl() {
        super();
        motor = new TalonFX(Ports.Funnel.MOTOR);
        Motors.Funnel.MOTOR_CONFIG.configure(motor);

        irSensor = new DigitalInput(Ports.Funnel.IR);

        hasCoral = BStream.create(irSensor).not()
                .filtered(new BDebounce.Rising(Settings.Funnel.HAS_CORAL_DEBOUNCE));

        shouldReverse = BStream.create(() -> motor.getSupplyCurrent().getValueAsDouble() > Settings.Funnel.STALL_CURRENT)
            .filtered(new BDebounce.Rising(Settings.Funnel.STALL_DETECTION_TIME))
            .filtered(new BDebounce.Falling(Settings.Funnel.MIN_REVERSE_TIME));
    }

    @Override
    public boolean hasCoral() {
        return hasCoral.get();
    }

    @Override
    public boolean shouldReverse() {
        return shouldReverse.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (Settings.EnabledSubsystems.FUNNEL.get()) {
            motor.set(getState().getSpeed());
        } 
        else {
            motor.set(0);
        }

        // SmartDashboard.putNumber("Funnel/Stator Current", motor.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Funnel/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
        
        SmartDashboard.putBoolean("Funnel/IR Sensor raw", irSensor.get());
        SmartDashboard.putBoolean("Funnel/Has Coral", hasCoral());
    }
}
