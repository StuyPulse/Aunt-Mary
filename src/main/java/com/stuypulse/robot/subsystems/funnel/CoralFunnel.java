package com.stuypulse.robot.subsystems.funnel;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class CoralFunnel extends SubsystemBase{
    
    private static final CoralFunnel instance;

    static {
        instance = new CoralFunnelImpl();
    }

    private final SmartNumber targetRPM;

    public static CoralFunnel getInstance() {
        return instance;
    }

    public CoralFunnel() {
        targetRPM = new SmartNumber("Funnel/Target RPM",  Settings.Funnel.TARGET_FUNNEL_RPM);
    }

    public double getTargetRPM() {
        return targetRPM.get();
    }

    public abstract void setMotorRPM(double targetRPM);

    public abstract boolean coralStuck();

    public abstract boolean getFunnelState();
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Funnel/Target RPM", getTargetRPM());
    }

}
