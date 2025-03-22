package com.stuypulse.robot.commands;

import com.stuypulse.robot.commands.shooter.ShooterAcquireAlgae;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL2Front;
import com.stuypulse.robot.commands.superStructure.algae.SuperStructureAlgaeL3Front;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveWithRobotRelativeSpeeds;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePIDToNearestReefAlgaePickup;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePIDToNearestReefAlgaeReady;
import com.stuypulse.robot.commands.swerve.pidToPose.algae.SwerveDrivePidToNearestReefAlgae;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure;
import com.stuypulse.robot.subsystems.superStructure.SuperStructure.SuperStructureState;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.Clearances;
import com.stuypulse.robot.util.ReefUtil;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ReefAlgaePickupRoutine extends SequentialCommandGroup{
    public ReefAlgaePickupRoutine() {
        addCommands(
            new ShooterAcquireAlgae(),
            new ConditionalCommand(
                new SwerveDrivePIDToNearestReefAlgaeReady(true)
                    .alongWith(new WaitUntilCommand(() -> Clearances.isArmClearFromReef()).andThen(new SuperStructureAlgaeL3Front()))
                    .alongWith(new WaitUntilCommand(() -> SuperStructure.getInstance().getState() == SuperStructureState.ALGAE_L3_FRONT && SuperStructure.getInstance().canSkipClearance()))
                    .andThen(new SwerveDrivePIDToNearestReefAlgaePickup(true))
                    .andThen(new SwerveDriveDriveWithRobotRelativeSpeeds(Settings.Swerve.NUDGE_SPEED_METERS_PER_SECOND, 0, 0)),
                new SwerveDrivePIDToNearestReefAlgaeReady(true)
                    .alongWith(new WaitUntilCommand(() -> Clearances.isArmClearFromReef()).andThen(new SuperStructureAlgaeL2Front()))
                    .alongWith(new WaitUntilCommand(() -> SuperStructure.getInstance().getState() == SuperStructureState.ALGAE_L2_FRONT && SuperStructure.getInstance().canSkipClearance()))
                    .andThen(new SwerveDrivePIDToNearestReefAlgaePickup(true))
                    .andThen(new SwerveDriveDriveWithRobotRelativeSpeeds(Settings.Swerve.NUDGE_SPEED_METERS_PER_SECOND, 0, 0)), 
                () -> ReefUtil.getClosestAlgae().isHighAlgae())
        );
    }
}
