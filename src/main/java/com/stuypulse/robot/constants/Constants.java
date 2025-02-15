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
        double MIN_HEIGHT_METERS = Units.inchesToMeters(40.85); // FROM FLOOR TO TOP OF ELEVATOR
        double MAX_HEIGHT_METERS = Units.inchesToMeters(69.85); // FROM FLOOR TO TOP OF ELEVATOR

        double WIDTH = Units.inchesToMeters(10.0); // Currently for simulation purposes only
        double FIXED_STAGE_MAX_HEIGHT = Units.inchesToMeters(40.748); // Currently for simulation purposes only
        double CARRIAGE_LENGTH = Units.inchesToMeters(35); // Currently for simulation purposes only

        double MASS_KG = 10.0;
        double DRUM_RADIUS_METERS = (MAX_HEIGHT_METERS / Encoders.NUM_ROTATIONS_TO_REACH_TOP * Encoders.GEAR_RATIO) / 2 / Math.PI;

        public interface Encoders {
            double GEAR_RATIO = 50.0 / 14.0;

            double NUM_ROTATIONS_TO_REACH_TOP = 16.4427978257; // Number of rotations that the motor has to spin, NOT the gear

            double POSITION_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP / 60;
        }
    }

    public interface Arm {
        double GEAR_RATIO = 50.0 / 3.0;

        double DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR = Units.inchesToMeters(5); // Current used for sim only

        Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);

        double ARM_LENGTH = Units.inchesToMeters(29);
        double MOMENT_OF_INERTIA = Units.lbsToKilograms(12.8) * ARM_LENGTH * ARM_LENGTH / 3;

        // Minimum angle at which the elevator can move up and down without hitting the funnel
        Rotation2d MIN_ANGLE_TO_CLEAR_FUNNEL_BACK = Rotation2d.fromDegrees(-180);
        Rotation2d MIN_ANGLE_TO_CLEAR_FUNNEL_FRONT = Rotation2d.fromDegrees(-93);

        Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-360);
        Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(360);
    }

    public interface Froggy {
        double GEAR_RATIO = 4.0 / 1.0;

        Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(0);

        Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
    }

    public interface Climb {
        double GEAR_RATIO = 75.0;
        Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
    }

    public interface LED {
        int LED_LENGTH = 0;
    }
}
