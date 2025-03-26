package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoral;
import com.stuypulse.robot.commands.shooter.ShooterShootAlgae;
import com.stuypulse.robot.commands.shooter.ShooterShootBasedOnSuperStructure;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;
import com.stuypulse.robot.subsystems.froggy.Froggy.RollerState;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class ManualShoot extends ConditionalCommand{
    public ManualShoot() {
        super(
            new ConditionalCommand(
                new FroggyRollerShootCoral(),
                new ShooterShootAlgae().onlyIf(() -> SuperStructure.getInstance().getState() == SuperStructureState.PROCESSOR)
                    .alongWith(new FroggyRollerShootAlgae().onlyIf(() -> Froggy.getInstance().getRollerState() != RollerState.HOLD_CORAL)),
                () -> Froggy.getInstance().getPivotState() == PivotState.L1_SCORE_ANGLE), 
            new ShooterShootBasedOnSuperStructure(),
            () -> Froggy.getInstance().getPivotState() == PivotState.L1_SCORE_ANGLE 
                || Froggy.getInstance().getPivotState() == PivotState.PROCESSOR_SCORE_ANGLE
                || SuperStructure.getInstance().getState() == SuperStructureState.PROCESSOR
        );
    }
}
