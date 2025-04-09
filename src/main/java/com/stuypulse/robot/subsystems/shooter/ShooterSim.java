
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.booleans.filters.BFilter;

import edu.wpi.first.math.geometry.Pose2d;

public class ShooterSim extends Shooter {

    private SmartBoolean hasCoral;
    private BStream hasShotLongEnoughToEjectCoral;

    protected ShooterSim() {
        super();
        hasCoral = new SmartBoolean("Shooter/Has Coral", true);
        hasShotLongEnoughToEjectCoral = BStream.create(() -> super.isShootingCoral()).filtered(new BDebounce.Rising(0.15));
    }

    @Override
    public boolean hasCoral() {
        return hasCoral.get();
    }

    @Override
    public boolean isAboveCoralCurrentThreshold() {
        return false;
    }

    @Override
    public void periodic() {
        super.periodic();
        
        if (hasShotLongEnoughToEjectCoral.get()) {
            hasCoral.set(false);
        }

        Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();
        Pose2d coralStationTargetPose = Field.CoralStation.getClosestCoralStation().getTargetPose();
        if (robotPose.getTranslation().getDistance(coralStationTargetPose.getTranslation()) < 0.2) {
            hasCoral.set(true);
        }
    }
}
