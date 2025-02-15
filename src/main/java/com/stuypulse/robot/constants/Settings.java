/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {

    double DT = 0.020;

    double TARGET_DISTANCE_FROM_REEF = 0.0;

    public interface LokiShooter {
        SmartNumber CORAL_SHOOT_SPEED = new SmartNumber("Coral Shoot Speed", 0.75);
        SmartNumber CORAL_ACQUIRE_SPEED = new SmartNumber("Coral Acquire Speed", 0.3);
        SmartNumber ALGAE_ACQUIRE_SPEED = new SmartNumber("Algae Acquire Speed", 0.45);
        SmartNumber ALGAE_SHOOT_SPEED = new SmartNumber("Algae Shoot Speed", 0.45);

        double HAS_CORAL_DEBOUNCE = 0.1;

        double STALL_DETECTION_DEBOUNCE = 0.5;
        double STALL_CURRENT_THRESHOLD = 30;
    }

    public interface Funnel {
        SmartNumber FORWARD_SPEED = new SmartNumber("Funnel/Forward Speed", 0.4);
        SmartNumber REVERSE_SPEED = new SmartNumber("Funnel/Reverse Speed", 0.4);

        double STALL_CURRENT = 30;
        double STALL_DETECTION_TIME = 0.25;
        double MIN_REVERSE_TIME = 1.0;

        double HAS_CORAL_DEBOUNCE = 0.5;
    }

    public interface Elevator {

        double MAX_VELOCITY_METERS_PER_SECOND = 0.0;
        double MAX_ACCEL_METERS_PER_SECOND_PER_SECOND = 0.0;

        double FEED_HEIGHT_METERS = 1.0;

        // Coral
        double FRONT_L2_HEIGHT_METERS = 1.609;
        double FRONT_L3_HEIGHT_METERS = 1.073;
        double FRONT_L4_HEIGHT_METERS = 1.708;
        
        double BACK_L2_HEIGHT_METERS = 1.106;
        double BACK_L3_HEIGHT_METERS = 1.161;
        double BACK_L4_HEIGHT_METERS = 1.836;

        // Algae
        double BARGE_HEIGHT_METERS = 2.073;
        double ALGAE_L2_HEIGHT_METERS = 1.4;
        double ALGAE_L3_HEIGHT_METERS = 1.8;

        double HEIGHT_TOLERANCE_METERS = 0.04;
    }

    public interface Arm {
        Rotation2d L2_ANGLE_FRONT = Rotation2d.fromDegrees(6.615 + 270);
        Rotation2d L3_ANGLE_FRONT = Rotation2d.fromDegrees(150.6 - 90);
        Rotation2d L4_ANGLE_FRONT = Rotation2d.fromDegrees(165.0 - 90);

        Rotation2d L2_ANGLE_BACK = Rotation2d.fromDegrees(261.5 - 90);
        Rotation2d L3_ANGLE_BACK = Rotation2d.fromDegrees(236.1 - 90);
        Rotation2d L4_ANGLE_BACK = Rotation2d.fromDegrees(240.0 - 90);

        Rotation2d ALGAE_L2_ANGLE = Rotation2d.fromDegrees(7);
        Rotation2d ALGAE_L3_ANGLE = Rotation2d.fromDegrees(9);
        Rotation2d BARGE_ANGLE = Rotation2d.fromDegrees(60.0);

        Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(-90);
        Rotation2d FEED_ANGLE = Rotation2d.fromDegrees(-93);

        double MAX_VEL_DEG_PER_S = 100.0;
        double MAX_ACCEL_DEG_PER_S_PER_S = 200.0;
        double ANGLE_TOLERANCE_DEGREES = 3.0;
    }

    public interface Froggy {
        Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d ALGAE_GROUND_PICKUP_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d CORAL_GROUND_PICKUP_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d GOLF_TEE_ALGAE_PICKUP_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d L1_SCORING_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d PROCESSOR_SCORE_ANGLE = Rotation2d.fromDegrees(0);

        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(1.0);

        double ALGAE_INTAKE_SPEED = 0.5;
        double ALGAE_OUTTAKE_SPEED = 0.5;

        double CORAL_INTAKE_SPEED = 0.5;
        double CORAL_OUTTAKE_SPEED = 0.5;

        double HOLD_ALGAE_SPEED = 0.0;

        double CORAL_STALL_CURRENT_THRESHOLD = 20.0;
        double ALGAE_STALL_CURRENT_THRESHOLD = 20.0;
        double STALL_DEBOUNCE_TIME = 0.0;

        double MAX_VEL_ROTATIONS_PER_S = 1.0;
        double MAX_ACCEL_ROTATIONS_PER_S_PER_S = 1.0;
    }

    public interface Climb {
        Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(0.0);
        Rotation2d OPEN_ANGLE = Rotation2d.fromDegrees(180.0);
        Rotation2d CLIMBED_ANGLE = Rotation2d.fromDegrees(270.0);
        
        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(2.0);
    }

    public interface LED {
        LEDPattern HAS_CORAL_COLOR = LEDPattern.solid(Color.kRed);

        LEDPattern ALIGN_COLOR = LEDPattern.solid(Color.kYellow);
        LEDPattern SHOOT_COLOR = LEDPattern.solid(Color.kGreen);
        LEDPattern ABORT_COLOR = LEDPattern.solid(Color.kBlue);

        LEDPattern INTAKE_COLOR = LEDPattern.rainbow(255, 255);

        LEDPattern FUNNEL_UNJAM_COLOR = LEDPattern.solid(Color.kBlue);

        LEDPattern L1_PROCESSOR_SCORING_COLOR = LEDPattern.solid(Color.kGreen);

        LEDPattern CLIMB_OPEN_COLOR = LEDPattern.rainbow(255, 255);

        LEDPattern CLIMBING_COLOR = LEDPattern.solid(Color.kGreen);
    }
}
