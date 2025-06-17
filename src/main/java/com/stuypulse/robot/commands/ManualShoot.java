package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootAlgae;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoralVersatile;
import com.stuypulse.robot.commands.shooter.ShooterShootBasedOnSuperStructure;
import com.stuypulse.robot.commands.shooter.scoring.ShooterShootAlgae;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.froggy.Froggy.PivotState;
import com.stuypulse.robot.subsystems.froggy.Froggy.RollerState;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;

public class ManualShoot extends ConditionalCommand {
    public 
    ManualShoot() {
        super(
            new ConditionalCommand(
                new FroggyRollerShootCoralVersatile(),
                new ShooterShootAlgae().onlyIf(() -> SuperStructure.getInstance().getState() == SuperStructureState.PROCESSOR || SuperStructure.getInstance().getState() == SuperStructureState.BARGE_118),
                () -> Froggy.getInstance().getPivotState() == PivotState.L1_SCORE_ANGLE_VERSATILE || 
                Froggy.getInstance().getPivotState() == PivotState.L1_SCORE_ANGLE_ONE ||
                Froggy.getInstance().getPivotState() == PivotState.L1_SCORE_ANGLE_TWO ||
                Froggy.getInstance().getPivotState() == PivotState.L1_SCORE_ANGLE_THREE), 
            new ShooterShootBasedOnSuperStructure(),
            () -> Froggy.getInstance().getPivotState() == PivotState.L1_SCORE_ANGLE_VERSATILE
                || SuperStructure.getInstance().getState() == SuperStructureState.PROCESSOR
                || SuperStructure.getInstance().getState() == SuperStructureState.BARGE_118
        );
    }
}
