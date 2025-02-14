/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public interface Constants {

    double LENGTH_WITH_BUMPERS_METERS = Units.inchesToMeters(30);
    double WIDTH_WITH_BUMPERS_METERS = Units.inchesToMeters(30);

    double SHOOTER_Y_OFFSET = Units.inchesToMeters(8.5);

    public interface Elevator {
        double MIN_HEIGHT_METERS = 0.889; // FROM FLOOR TO TOP OF ELEVATOR
        double MAX_HEIGHT_METERS = 2.1; // FROM FLOOR TO TOP OF ELEVATOR

        double MASS_KG = 10.0;
        double DRUM_RADIUS_METERS = (MAX_HEIGHT_METERS / Encoders.NUM_ROTATIONS_TO_REACH_TOP * Encoders.GEAR_RATIO) / 2 / Math.PI;

        public interface Encoders {
            double GEAR_RATIO = 1.0 / 5.0;

            double NUM_ROTATIONS_TO_REACH_TOP = (6 + 9.0 / 24) * GEAR_RATIO; // Number of rotations that the motor has to spin, NOT the gear

            double POSITION_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP / 60;
        }
    }

    public interface Arm {
        double GEAR_RATIO = 0.833333330333;

        Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);

        double ARM_LENGTH = Units.inchesToMeters(3);
        double MOMENT_OF_INERTIA = Units.lbsToKilograms(20) * ARM_LENGTH * ARM_LENGTH / 3;

        Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-360);
        Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(360);
    }

    public interface Froggy {
        double GEAR_RATIO = 0.0;

        Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(0);

        Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
    }

    public interface Climb {
        double GEAR_RATIO = 25;
        Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
    }

    public interface LED {
        int LED_LENGTH = 0;
    }
}
