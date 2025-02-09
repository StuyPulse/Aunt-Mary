/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.funnel;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;

public class FunnelImpl extends Funnel {

    private final TalonFX motor;

    private final DigitalInput irSensor;
    private final BStream hasCoral;
    private final BStream isStalling;

    public FunnelImpl() {
        motor = new TalonFX(Ports.Funnel.MOTOR);
        Motors.Funnel.MOTOR_CONFIG.configure(motor);

        irSensor = new DigitalInput(Ports.Funnel.IR);

        hasCoral = BStream.create(irSensor).not()
                    .filtered(new BDebounce.Rising(Settings.Funnel.HAS_CORAL_DEBOUNCE));
                    
        isStalling = BStream.create(() -> Math.abs(motor.getSupplyCurrent().getValueAsDouble()) > Settings.Funnel.STALL_CURRENT)
                        .filtered(new BDebounce.Rising(Settings.Funnel.STALL_DETECTION_TIME));
    }

    @Override
    public void forward() {
        motor.set(Settings.Funnel.FORWARD_SPEED.get());
    }

    @Override
    public void reverse() {
        motor.set(-Settings.Funnel.REVERSE_SPEED.get());
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public boolean isStalling() {
        return isStalling.get();
    }

    @Override
    public boolean hasCoral() {
        return hasCoral.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Funnel/Current", motor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Funnel/Has Coral", hasCoral());
        SmartDashboard.putBoolean("Funnel/Is Stalling", isStalling());
    }
}
