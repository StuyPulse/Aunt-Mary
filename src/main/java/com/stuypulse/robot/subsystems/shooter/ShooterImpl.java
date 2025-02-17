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

    private final DigitalInput IRSensor;
    private final BStream hasCoral;
    private final BStream isStalling;

    protected ShooterImpl() {
        super();
        motor = new TalonFX(Ports.Shooter.MOTOR);
        Motors.LokiShooter.MOTOR_CONFIG.configure(motor);

        IRSensor = new DigitalInput(Ports.Shooter.IR_SENSOR);

        hasCoral = BStream.create(IRSensor).not()
                    .filtered(new BDebounce.Both(Settings.Shooter.HAS_CORAL_DEBOUNCE));

        isStalling = BStream.create(() -> Math.abs(motor.getStatorCurrent().getValueAsDouble()) > Settings.Shooter.STALL_CURRENT_THRESHOLD)
                    .filtered(new BDebounce.Both(Settings.Shooter.STALL_DETECTION_DEBOUNCE));
    }

    @Override
    public boolean hasCoral() {
        return hasCoral.get();
    }

    @Override
    public boolean isStalling() {
        return isStalling.get();
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

        SmartDashboard.putNumber("Shooter/Voltage", motor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Current", motor.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Shooter/Has Coral", hasCoral());
        SmartDashboard.putBoolean("Shooter/Is Stalling", isStalling());
    }
}
