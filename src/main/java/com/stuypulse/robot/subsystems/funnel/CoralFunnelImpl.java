/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.funnel;

import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.IStream;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CoralFunnelImpl extends CoralFunnel {

    private final TalonFX funnelMotor;
    private final TalonFXConfiguration funnelConfig;

    private final DigitalInput irSensor;
    private final BStream hasCoral;
    private final BStream isStalling;

    private final IStream funnelCurrent;

    public CoralFunnelImpl() {
        funnelMotor = new TalonFX(Ports.Funnel.MOTOR);

        irSensor = new DigitalInput(Ports.Funnel.IR);

        // Stall Detection
        funnelCurrent =
                IStream.create(() -> Math.abs(funnelMotor.getStatorCurrent().getValueAsDouble()));

        hasCoral =
                BStream.create(irSensor)
                        .not()
                        .filtered(new BDebounce.Both(Settings.Funnel.IR_DEBOUNCE));
        isStalling =
                BStream.create(() -> funnelCurrent.get() > Settings.Funnel.DRIVE_CURRENT_THRESHOLD)
                        .filtered(new BDebounce.Both(Settings.Funnel.FUNNEL_STALLING));

        funnelConfig = new TalonFXConfiguration();

        funnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        funnelConfig.Feedback.SensorToMechanismRatio = Settings.Funnel.GEAR_RATIO;
        funnelConfig.TorqueCurrent.PeakForwardTorqueCurrent = Settings.Funnel.DRIVE_CURRENT_LIMIT;
        funnelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Settings.Funnel.DRIVE_CURRENT_LIMIT;
        funnelConfig.CurrentLimits.StatorCurrentLimit = Settings.Funnel.DRIVE_CURRENT_LIMIT;
        funnelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Ramp Motor Voltage
        funnelConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Settings.Funnel.RAMP_RATE;
        funnelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.Funnel.RAMP_RATE;

        funnelMotor.getConfigurator().apply(funnelConfig);
    }

    @Override
    public void forward() {
        funnelMotor.set(Settings.Funnel.MOTOR_SPEED.getAsDouble());
    }

    @Override
    public void reverse() {
        funnelMotor.set(-Settings.Funnel.MOTOR_SPEED.getAsDouble());
    }

    @Override
    public void stop() {
        funnelMotor.set(0);
    }

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

        SmartDashboard.putNumber(
                "Funnel/Voltage", funnelMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Funnel/Current", funnelCurrent.get());

        SmartDashboard.putBoolean("Funnel/Has Coral", hasCoral());
        SmartDashboard.putBoolean("Funnel/Coral IsStuck", isStalling());
    }
}
