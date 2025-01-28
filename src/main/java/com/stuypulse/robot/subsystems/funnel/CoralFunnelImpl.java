package com.stuypulse.robot.subsystems.funnel;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;

public class CoralFunnelImpl extends CoralFunnel {
    
    private final TalonFX driveMotor;
    private final DigitalInput motorBeam;
    private boolean funnelState;

    public CoralFunnelImpl(){
        driveMotor = new TalonFX(Ports.Funnel.MOTOR);
        motorBeam = new DigitalInput(Ports.Funnel.BEAM);
    }

    @Override
    public void periodic() {
        super.periodic();

    
    }
}
