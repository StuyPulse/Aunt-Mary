/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.froggy.roller;

import com.stuypulse.robot.subsystems.froggy.*;
import com.stuypulse.robot.subsystems.froggy.Froggy.RollerState;

import edu.wpi.first.wpilibj2.command.Command;

public class FroggyRollerIntakeAlgae extends Command {

    private final Froggy froggy;

    public FroggyRollerIntakeAlgae() {
        froggy = Froggy.getInstance();
        addRequirements(froggy);
    }

    @Override
    public void initialize() {
        froggy.setRollerState(RollerState.INTAKE_ALGAE);
    }

    @Override
    public boolean isFinished() {
        return froggy.isStalling();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            froggy.setRollerState(RollerState.HOLD_ALGAE);
        }
        else {
            froggy.setRollerState(RollerState.STOP);
        }
    }
}
