/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.020; // 20ms Differential Time

    public interface Shooter {
        SmartNumber CORAL_FRONT_SPEED = new SmartNumber("Coral Target Speed", 0.75);
        SmartNumber CORAL_BACK_SPEED = new SmartNumber("Coral Target Speed", 0.75);
        SmartNumber CORAL_ACQUIRE_SPEED = new SmartNumber("Coral Acquire Speed", 0.3);
        SmartNumber ALGAE_ACQUIRE_SPEED = new SmartNumber("Algae Acquire Speed", 0.45);
        SmartNumber ALGAE_SHOOT_SPEED = new SmartNumber("Algae Shoot Speed", 0.45);

        double BB_DEBOUNCE = 0.0;
        double CORAL_STALLING_DEBOUNCE = 0.0;
        double ALGAE_DEBOUNCE = 0.0;

        double RAMP_RATE = 0.0;

        double DRIVE_CURRENT_THRESHOLD = 30;
        double DRIVE_CURRENT_LIMIT = 40;
    }

    public interface Funnel {
        SmartNumber MOTOR_SPEED = new SmartNumber("Funnel Speed", 0.0);
        double IR_DEBOUNCE = 0.0;
        double FUNNEL_STALLING = 0.0;

        double RAMP_RATE = 0.0;

        double GEAR_RATIO = 0.0;
        double DRIVE_CURRENT_THRESHOLD = 30;
        double DRIVE_CURRENT_LIMIT = 40;
    }

    public interface Elevator {
        SmartNumber MAX_VELOCITY_METERS_PER_SECOND =
                new SmartNumber("Elevator/Max Velocity (m per s)", 1.0);
        SmartNumber MAX_ACCEL_METERS_PER_SECOND_PER_SECOND =
                new SmartNumber("Elevator/Max Accel (m per s^2)", 2.0);

        // CHANGE
        double HANDOFF_HEIGHT_METERS = 0.1;
        // front and funnel
        double L2_FRONT_HEIGHT = 0.25;
        double L3_FRONT_HEIGHT = 0.5;
        double L4_FRONT_HEIGHT = 0.75;
        double L2_BACK_HEIGHT = 0.3; // funnel side; should be higher than L2
        double L3_BACK_HEIGHT = 0.55; // funnel side; should be higher than L3
        double L4_BACK_HEIGHT = 0.8;
        double L2_REEF_HEIGHT = 0.5;
        double L3_REEF_HEIGHT = 0.75;

        double RAMP_RATE_CURRENT = 1.0;
        double RAMP_RATE_VOLTAGE = 0.1;

        // FIND OUT REAL GEAR RATIO
        double GEAR_RATIO = 1.0 / 5.0;
        double CURRENT_LIMIT = 5.0;

        // Magic motion, change RPS
        double TARGET_CRUISE_VELOCITY = 0.0; // Rotations Per Second
        double TARGET_ACCELERATION = 0.0; // Rotations Per Second^2
        double TARGET_JERK = 0.0; // Rotations Per Second^3

        SmartNumber HEIGHT_TOLERANCE_METERS =
                new SmartNumber("Elevator/Height Tolerance (m)", 0.02);

        public interface PID {
            // tune
            SmartNumber kP = new SmartNumber("Elevator/Controller/kP", 0.0);
            SmartNumber kI = new SmartNumber("Elevator/Controller/kI", 0.0);
            SmartNumber kD = new SmartNumber("Elevator/Controller/kD", 0.0);
        }

        public interface FF {
            SmartNumber kS = new SmartNumber("Elevator/Controller/kS", 0.20506);
            SmartNumber kV = new SmartNumber("Elevator/Controller/kV", 3.7672);
            SmartNumber kA = new SmartNumber("Elevator/Controller/kA", 0.27);
            SmartNumber kG = new SmartNumber("Elevator/Controller/kG", 1.37);
        }

        public interface Simulation {
            double SCALE_FACTOR = 0.5 + 2.5 / 77;
        }
    }

    public interface Arm {

        public interface PID {
            SmartNumber kP = new SmartNumber("Arm/PID/kP", 0.0);
            SmartNumber kI = new SmartNumber("Arm/PID/kI", 0);
            SmartNumber kD = new SmartNumber("Arm/PID/kD", 0);
        }

        public interface FF {
            SmartNumber kS = new SmartNumber("Arm/PID/kP", 0.0);
            SmartNumber kV = new SmartNumber("Arm/PID/kI", 0);
            SmartNumber kA = new SmartNumber("Arm/PID/kD", 0);
            SmartNumber kG = new SmartNumber("Arm/PID/kG", 0);
        }

        Rotation2d L2_ANGLE_FRONT = Rotation2d.fromDegrees(0);
        Rotation2d L3_ANGLE_FRONT = Rotation2d.fromDegrees(0);
        Rotation2d L4_ANGLE_FRONT = Rotation2d.fromDegrees(0);

        Rotation2d L2_ANGLE_BACK = Rotation2d.fromDegrees(0);
        Rotation2d L3_ANGLE_BACK = Rotation2d.fromDegrees(0);
        Rotation2d L4_ANGLE_BACK = Rotation2d.fromDegrees(0);

        Rotation2d FUNNEL_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d BARGE_ANGLE = Rotation2d.fromDegrees(0);
        
        double ARM_OFFSET = 0;
        double ENCODER_OFFSET = 0;
        SmartNumber PID_RAMPING = new SmartNumber("Arm/PID_RAMP", 0);
        SmartNumber FF_RAMPING = new SmartNumber("Arm/FF_RAMP", 0);
        SmartNumber CURRENT_RAMPING = new SmartNumber("Arm/CURRENT_RAMP", 0);

        double GEAR_RATIO = 0.833333330333;
        double AREA = 3; // meters squared
        double ARM_LENGTH = Units.inchesToMeters(3);
        double MOMENT_OF_INERTIA = Units.lbsToKilograms(20) * ARM_LENGTH * ARM_LENGTH / 3;

        double LOWER_ANGLE_LIMIT = 0;
        double UPPER_ANGLE_LIMIT = 360;

        double MAX_VELOCITY_METERS_PER_SECOND = 0;
        double MAX_ACCEL_METERS_PER_SECOND_PER_SECOND = 0;

        public interface MotionMagic {
            double MAX_VEL = 0;
            double MAX_ACCEL = 0;
            double JERK = 0;
        }
    }

    public interface Froggy {
        Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d ALGAE_GROUND_PICKUP_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d CORAL_GROUND_PICKUP_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d GOLF_TEE_ALGAE_PICKUP_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d L1_SCORING_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d PROCESSOR_SCORE_ANGLE = Rotation2d.fromDegrees(0);

        double ALGAE_INTAKE_SPEED = 0.0;
        double ALGAE_OUTTAKE_SPEED = 0.0;
        double CORAL_INTAKE_SPEED = 0.0;
        double CORAL_OUTTAKE_SPEED = 0.0;

        double CORAL_CURRENT_THRESHOLD = 0.0;
        double ALGAE_CURRENT_THRESHOLD = 0.0;

        double GEAR_RATIO = 0.0;
        double PIVOT_CURRENT_LIMIT = 0.0;
        double STATOR_CURRENT_LIMIT = 0.0;
        double ROLLER_CURRENT_THRESHOLD = 0.0;
        double ANGLE_OFFSET = 0.0;

        double ROLLER_FF_RAMPING = 0.0;
        double PIVOT_FF_RAMPING = 0.0;
        double ROLLER_PID_RAMPING = 0.0;
        double PIVOT_PID_RAMPING = 0.0;
        double ROLLER_DEBOUNCE_TIME = 0.0;

        double MINIMUM_ANGLE = 0.0;
        double MAXIMUM_ANGLE = 0.0;
        double MAGNET_OFFSET = 0.0;
        double ENCODER_GEAR_RATIO = 0.0;
        double HOLD_SPEED = 0.0;

        public interface PID {
            SmartNumber kP = new SmartNumber("kP", 0.0);
            SmartNumber kI = new SmartNumber("kI", 0.0);
            SmartNumber kD = new SmartNumber("kD", 0.0);
            SmartNumber kG = new SmartNumber("kG", 0.0);
        }

        public interface FF {
            SmartNumber kA = new SmartNumber("kA", 0.0);
            SmartNumber kS = new SmartNumber("kS", 0.0);
            SmartNumber kV = new SmartNumber("kV", 0.0);
        }

        public interface MotionMagic {
            double MAX_VELOCITY = 0.0;
            double MAX_ACCELERATION = 0.0;
            double MAX_JERK = 0.0;
        }
    }

    public interface Climb {
        double kS = 0.0;
        double kV = 0.0;
        double kA = 0.0;
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;

        int CURRENT_LIMIT = 0;
        double GEAR_RATIO = 25;
        double RAMP_RATE = 0;

        double STOW_ANGLE = 0.0;
        double INTAKE_ANGLE = 0.0;
        double ACQUIRED_ANGLE = 0.0;
        double CLIMBED_ANGLE = 0.0;
        double CLIMB_ANGLE_TOLERANCE = 2.0;
    }
    
    public interface LED {
        public int LED_LENGTH = 0;
    }

}
