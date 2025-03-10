package com.stuypulse.robot.commands.arm.coral;

import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;

public class ArmToL1 extends ArmSetState{
    public ArmToL1(){
        super(ArmState.L1);
    }
}
