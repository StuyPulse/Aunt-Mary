package com.stuypulse.robot.subsystems.funnel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.IStream;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralFunnelImpl extends CoralFunnel {
    
    private final TalonFX funnelMotor;
    private final TalonFXConfiguration funnelConfig;

    private final DigitalInput motorBeam;
    private BStream hasCoral;
    private BStream isStalling;

    private final IStream funnelCurrent;

    public CoralFunnelImpl(){
        funnelMotor = new TalonFX(Ports.Funnel.MOTOR);

        motorBeam = new DigitalInput(Ports.Funnel.BEAM);

        //Stall Detection
        funnelCurrent = IStream.create(() -> Math.abs(funnelMotor.getStatorCurrent().getValueAsDouble()));

        hasCoral = BStream.create(motorBeam).not()
            .filtered(new BDebounce.Both(Settings.Funnel.BB_DEBOUNCE));
        isStalling = BStream.create(() -> funnelCurrent.get() > Settings.Funnel.DRIVE_CURRENT_THRESHOLD)
            .filtered(new BDebounce.Both(Settings.Funnel.FUNNEL_STALLING));

        funnelConfig = new TalonFXConfiguration();

        funnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        funnelConfig.Feedback.SensorToMechanismRatio = Settings.Funnel.GEAR_RATIO;
        funnelConfig.TorqueCurrent.PeakForwardTorqueCurrent = Settings.Funnel.DRIVE_CURRENT_LIMIT;
        funnelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Settings.Funnel.DRIVE_CURRENT_LIMIT;
        funnelConfig.CurrentLimits.StatorCurrentLimit = Settings.Funnel.DRIVE_CURRENT_LIMIT;
        funnelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        //Ramp Motor Voltage
        funnelConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Settings.Funnel.RAMP_RATE;
        funnelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.Funnel.RAMP_RATE;
        
        funnelMotor.getConfigurator().apply(funnelConfig);
    }

    @Override
    public void forward() {
        this.funnelMotor.set(Settings.Funnel.MOTOR_SPEED.getAsDouble());
    }

    @Override
    public void reverse() {
        this.funnelMotor.set(-Settings.Funnel.MOTOR_SPEED.getAsDouble());
    }

    @Override
    public void stop() {
        this.funnelMotor.set(0);
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

        SmartDashboard.putNumber("Funnel/Voltage", funnelMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Funnel/Current", funnelCurrent.get());

        SmartDashboard.putBoolean("Funnel/Has Coral", hasCoral());
        SmartDashboard.putBoolean("Funnel/Coral IsStuck", isStalling());
    }
}
