/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.util.Units;

public class Constants {
    public interface Elevator {
        double MIN_HEIGHT_METERS = 0.889; // check?
        double MAX_HEIGHT_METERS = 2.1; // definitely check
        // double MIN_HEIGHT_METERS =
        //         Units.inchesToMeters(9.09375); // FROM THE BOTTOM OF FIXED STAGE TO TOP OF CARRIAGE
        // double MAX_HEIGHT_METERS =
        //         Units.inchesToMeters(77); // FROM THE BOTTOM OF FIXED STAGE TO TOP ELEVATOR

        double MASS_KG = 10.0;
        double DRUM_RADIUS_METERS =
                (MAX_HEIGHT_METERS / Encoders.NUM_ROTATIONS_TO_REACH_TOP * Encoders.GEARING)
                        / 2 / Math.PI;

        // FIND OUT REAL GEAR RATIO
        double GEAR_RATIO = 1.0 / 5.0;


        public interface Encoders {
            double GEARING = 4.0;

            double NUM_ROTATIONS_TO_REACH_TOP =
                    (6 + 9.0 / 24)
                            * GEARING; // Number of rotations that the motor has to spin, NOT the gear

            double POSITION_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP / 60;

            double DISTANCE_PER_ROTATION = 0.0;
        }
    }

        public interface Arm {

            double GEAR_RATIO = 0.833333330333;

            double ANGLE_OFFSET = 0.0;
            double DELTA_MAX_ANGLE = 40.0; // Max angle clearance for arm funnel side

            double AREA = 3; // meters squared
            double ARM_LENGTH = Units.inchesToMeters(3);
            double MOMENT_OF_INERTIA = Units.lbsToKilograms(20) * ARM_LENGTH * ARM_LENGTH / 3;
            
            double LOWER_ANGLE_LIMIT = 0;
            double UPPER_ANGLE_LIMIT = 360;
        }

        public interface Froggy {
            double GEAR_RATIO = 0.0;
            double MINIMUM_ANGLE = 0.0;
            double MAXIMUM_ANGLE = 0.0;
            double MAGNET_OFFSET = 0.0;
            double ENCODER_GEAR_RATIO = 0.0;
            double ANGLE_OFFSET = 0.0;
        }

        public interface Climb {
            double GEAR_RATIO = 25;
        }

        public interface LED {
            int LED_LENGTH = 0;
        }
}
