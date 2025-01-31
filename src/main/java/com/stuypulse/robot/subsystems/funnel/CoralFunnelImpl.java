package com.stuypulse.robot.subsystems.funnel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.HighPassFilter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralFunnelImpl extends CoralFunnel {
    
    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveConfig;

    private final DigitalInput motorBeam;
    private BStream funnelState;

    private final IStream driveCurrent;

    public CoralFunnelImpl(){
        driveMotor = new TalonFX(Ports.Funnel.MOTOR);

        motorBeam = new DigitalInput(Ports.Funnel.BEAM);
        funnelState = BStream.create(motorBeam).not()
            .filtered(new BDebounce.Both(Settings.Funnel.BB_DEBOUNCE));

        driveConfig = new TalonFXConfiguration();

        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = Settings.Funnel.FeedForward.kS.getAsDouble();
        slot0.kV = Settings.Funnel.FeedForward.kV.getAsDouble();
        slot0.kA = Settings.Funnel.FeedForward.kA.getAsDouble();
        
        slot0.kP = Settings.Funnel.PID.kP.getAsDouble();
        slot0.kI = Settings.Funnel.PID.kI.getAsDouble();
        slot0.kD = Settings.Funnel.PID.kD.getAsDouble();

        driveConfig.Slot0 = slot0;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.SensorToMechanismRatio = Settings.Funnel.GEAR_RATIO;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = Settings.Funnel.DRIVE_CURRENT_LIMIT;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Settings.Funnel.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.StatorCurrentLimit = Settings.Funnel.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        driveConfig.TorqueCurrent.TorqueNeutralDeadband = 0.05;

        //Stall Detection
        driveCurrent = IStream.create(() -> driveMotor.getStatorCurrent().getValueAsDouble())
            .filtered(new HighPassFilter(Settings.Funnel.DRIVE_CURRENT_THRESHOLD));
        
        driveMotor.getConfigurator().apply(driveConfig);
        driveMotor.setPosition(0);

        //Ramp Motor Voltage
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Settings.Funnel.MAX_FUNNEL_RPM;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Settings.Funnel.MAX_FUNNEL_RPM;
    }

    private double getMotorRPM() {
        return driveMotor.get() * Settings.Funnel.MAX_FUNNEL_RPM;
    }

    public void setMotorRPM(double targetRPM) {
        driveMotor.set(targetRPM / Settings.Funnel.MAX_FUNNEL_RPM);
    }

    public boolean coralStuck() {
        return driveCurrent.get() > Settings.Funnel.DRIVE_CURRENT_THRESHOLD;
    }

    @Override
    public boolean getFunnelState() {
        return funnelState.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Funnel/RPM", getMotorRPM());

        SmartDashboard.putNumber("Funnel/Voltage", driveMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Funnel/Current", driveCurrent.get());

        SmartDashboard.putBoolean("Funnel/Has Coral", getFunnelState());
        SmartDashboard.putBoolean("Funnel/Coral Stuck", coralStuck());
    }
}
