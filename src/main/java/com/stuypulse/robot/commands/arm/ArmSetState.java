package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmSetState extends InstantCommand{
    private final Arm arm;
    private final ArmState state;

    public ArmSetState(ArmState state) {
        this.arm = Arm.getInstance();
        this.state = state;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setState(state);
    }
}
