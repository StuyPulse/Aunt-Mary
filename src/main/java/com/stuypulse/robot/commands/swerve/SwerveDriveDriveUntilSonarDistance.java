
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.froggy.Froggy;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDriveUntilSonarDistance extends Command {

    private final CommandSwerveDrivetrain swerve;

    private double sonarDistance;

    public SwerveDriveDriveUntilSonarDistance() {
        swerve = CommandSwerveDrivetrain.getInstance();
        sonarDistance = Froggy.getInstance().getSonarDistanceInches();

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        if (sonarDistance - Settings.Swerve.Alignment.Tolerances.SONAR_DISTANCE_TOLERANCE > 0) {
            swerve.setControl(swerve.getRobotCentricSwerveRequest()
            .withVelocityX(0)
            .withVelocityY(Settings.Swerve.NUDGE_SPEED_METERS_PER_SECOND_SONAR)
            .withRotationalRate(0));
        } else {
            swerve.setControl(swerve.getRobotCentricSwerveRequest()
            .withVelocityX(0)
            .withVelocityY(-Settings.Swerve.NUDGE_SPEED_METERS_PER_SECOND_SONAR)
            .withRotationalRate(0));
        }
    }
}