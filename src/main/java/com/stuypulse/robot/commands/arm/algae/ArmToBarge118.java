package com.stuypulse.robot.commands.arm.algae;

import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;

public class ArmToBarge118 extends ArmSetState {
    public ArmToBarge118() {
        super(ArmState.BARGE_118);
    }
}
