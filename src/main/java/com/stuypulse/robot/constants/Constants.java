package com.stuypulse.robot.constants;

import edu.wpi.first.math.util.Units;

public class Constants {
    public interface Elevator {
        double MIN_HEIGHT_METERS = Units.inchesToMeters(9.09375); // FROM THE BOTTOM OF FIXED STAGE TO TOP OF CARRIAGE
        double MAX_HEIGHT_METERS = Units.inchesToMeters(77); // FROM THE BOTTOM OF FIXED STAGE TO TOP ELEVATOR

        double MASS_KG = 10.0;
        double DRUM_RADIUS_METERS = (MAX_HEIGHT_METERS / Encoders.NUM_ROTATIONS_TO_REACH_TOP * Encoders.GEARING) / 2 / Math.PI;

        public interface Encoders {
            double GEARING = 4.0;

            double NUM_ROTATIONS_TO_REACH_TOP = (6 + 9.0 / 24) * GEARING; // Number of rotations that the motor has to spin, NOT the gear

            double POSITION_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP / 60;

            double DISTANCE_PER_ROTATION = 0.0;
        }
    }
}
