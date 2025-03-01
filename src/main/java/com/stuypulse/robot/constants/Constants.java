/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public interface Constants {

    double LENGTH_WITH_BUMPERS_METERS = Units.inchesToMeters(37.16);
    double WIDTH_WITH_BUMPERS_METERS = Units.inchesToMeters(36.16);
    
    double SHOOTER_Y_OFFSET = Units.inchesToMeters(3.5);

    public interface Elevator {
        double MIN_HEIGHT_METERS = Units.inchesToMeters(40.85); // FROM FLOOR TO TOP OF ELEVATOR
        double MAX_HEIGHT_METERS = Units.inchesToMeters(69.85); // FROM FLOOR TO TOP OF ELEVATOR

        double WIDTH = Units.inchesToMeters(10.0); // Currently for simulation purposes only
        double FIXED_STAGE_MAX_HEIGHT = Units.inchesToMeters(40.748); // Currently for simulation purposes only
        double CARRIAGE_LENGTH = Units.inchesToMeters(35); // Currently for simulation purposes only

        double MASS_KG = Units.lbsToKilograms(10) + Arm.MASS_KG; // Currently for sim only, not confirmed
        double DRUM_RADIUS_METERS = ((MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (Encoders.NUM_ROTATIONS_TO_REACH_TOP / Encoders.GEAR_RATIO)) / 2 / Math.PI;

        public interface Encoders {
            double GEAR_RATIO = 50.0 / 14.0;

            double NUM_ROTATIONS_TO_REACH_TOP = 16.4427978257; // Number of rotations that the motor has to spin, NOT the gear

            double POSITION_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP / 60;
        }
    }

    public interface Arm {
        double GEAR_RATIO = 30.0;

        double DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR = Units.inchesToMeters(5); // Current used for sim only

        Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(201.848576 + 90);

        double ARM_LENGTH = Units.inchesToMeters(29);
        double MASS_KG = Units.lbsToKilograms(12.8);
        double MOMENT_OF_INERTIA = MASS_KG * ARM_LENGTH * ARM_LENGTH / 3;
    }

    public interface Shooter {
        double DISTANCE_FROM_ARM_PIVOT_TO_TOP_ROLLER = Units.inchesToMeters(15); // Current used for sim only
        double DISTANCE_FROM_ARM_PIVOT_TO_BOTTOM_ROLLER = Units.inchesToMeters(22); // Current used for sim only
    }

    public interface Froggy {
        double GEAR_RATIO = 48.0;

        Rotation2d MINIMUM_ANGLE = Rotation2d.fromDegrees(-51);
        Rotation2d MAXIMUM_ANGLE = Rotation2d.fromDegrees(105);

        Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(219.815204 - 90);
    }

    public interface Climb {
        double GEAR_RATIO = 75.0;
        Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(259 - 60);

        Rotation2d MIN_ANGLE = Rotation2d.kZero;
        Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(265);
    }

    public interface LED {
        int LED_LENGTH = 50;
    }
}
