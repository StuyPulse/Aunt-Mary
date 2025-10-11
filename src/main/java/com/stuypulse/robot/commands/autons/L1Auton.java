package com.stuypulse.robot.commands.autons;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.froggy.pivot.FroggyPivotToL1One;
import com.stuypulse.robot.commands.froggy.roller.FroggyRollerShootCoralOne;
import com.stuypulse.robot.commands.swerve.pidToPose.coral.SwerveDrivePIDToClosestL1FroggyScore;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class L1Auton extends SequentialCommandGroup {
    
    public L1Auton(PathPlannerPath... paths) {

        addCommands(

            new SwerveDrivePIDToClosestL1FroggyScore(2).alongWith(new FroggyPivotToL1One())
                .andThen(new FroggyRollerShootCoralOne())

        );

    }

}
