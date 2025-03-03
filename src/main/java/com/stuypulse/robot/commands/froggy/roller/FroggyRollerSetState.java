package com.stuypulse.robot.commands.froggy.roller;

import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.froggy.Froggy.RollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FroggyRollerSetState extends InstantCommand{
    private final Froggy froggy;
    private final RollerState rollerState;

    public FroggyRollerSetState(RollerState state) {
        this.froggy = Froggy.getInstance();
        this.rollerState = state;
    }

    @Override
    public void initialize() {
        froggy.setRollerState(rollerState);
    }
}
