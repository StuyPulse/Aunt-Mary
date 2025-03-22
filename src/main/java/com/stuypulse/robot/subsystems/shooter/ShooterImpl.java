/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {

    private final TalonFX motor;

    private final DigitalInput beamBreak;
    private final BStream hasCoral;

    protected ShooterImpl() {
        super();
        motor = new TalonFX(Ports.Shooter.MOTOR);
        Motors.Shooter.MOTOR_CONFIG.configure(motor);

        beamBreak = new DigitalInput(Ports.Shooter.BEAM_BREAK);

        hasCoral = BStream.create(beamBreak).not()
                    .filtered(new BDebounce.Both(Settings.Shooter.HAS_CORAL_DEBOUNCE));
    }

    @Override
    public boolean hasCoral() {
        return hasCoral.get();
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
            SmartDashboard.putNumber("Shooter/Voltage", motor.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Shooter/Supply Current", motor.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Shooter/Stator Current", motor.getStatorCurrent().getValueAsDouble());
        }
    }
}
