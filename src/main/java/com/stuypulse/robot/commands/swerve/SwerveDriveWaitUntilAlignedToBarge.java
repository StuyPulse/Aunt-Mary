package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveWaitUntilAlignedToBarge extends Command{
    private final CommandSwerveDrivetrain swerve;
    private final BStream isAligned;

    public SwerveDriveWaitUntilAlignedToBarge() {
        this.swerve = CommandSwerveDrivetrain.getInstance();

        isAligned = BStream.create(swerve::isAlignedToBargeX)
            .filtered(new BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE));
    }

    @Override
    public boolean isFinished() {
        return isAligned.get();
    }
}
