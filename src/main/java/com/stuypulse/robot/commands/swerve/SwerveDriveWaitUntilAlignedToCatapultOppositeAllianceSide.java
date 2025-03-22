package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Alignment;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWaitUntilAlignedToCatapultOppositeAllianceSide extends Command{
    private final BStream isAligned;

    public SwerveDriveWaitUntilAlignedToCatapultOppositeAllianceSide() {
        isAligned = BStream.create(this::isAligned)
            .filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE));
    }

    private boolean isAligned() {
        return Math.abs((Field.LENGTH / 2 + Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_CATAPULT) - CommandSwerveDrivetrain.getInstance().getPose().getX())
            < Alignment.Tolerances.X_TOLERANCE;
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }
}
