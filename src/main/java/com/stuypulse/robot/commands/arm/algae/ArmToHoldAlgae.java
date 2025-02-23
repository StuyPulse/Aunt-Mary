package com.stuypulse.robot.commands.arm.algae;

import com.stuypulse.robot.commands.arm.ArmSetState;
import com.stuypulse.robot.subsystems.arm.Arm;

public class ArmToHoldAlgae extends ArmSetState {
    public ArmToHoldAlgae() {
        super(Arm.ArmState.HOLD_ALGAE);
    }
}
