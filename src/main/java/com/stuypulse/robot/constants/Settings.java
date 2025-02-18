/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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

    String CANIVORE_NAME = "CANIVORE";

    double CLEARANCE_DISTANCE_FROM_REEF = 0.4;
    double CLEARANCE_DISTANCE_FROM_CENTERLINE_FOR_BARGE = Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE + 0.4;

    public interface EnabledSubsystems {
        SmartBoolean SWERVE = new SmartBoolean("Enabled Subsystems/Swerve Is Enabled", false);
        SmartBoolean ARM = new SmartBoolean("Enabled Subsystems/Arm Is Enabled", false);
        SmartBoolean ELEVATOR = new SmartBoolean("Enabled Subsystems/Elevator Is Enabled", false);
        SmartBoolean SHOOTER = new SmartBoolean("Enabled Subsystems/Shooter Is Enabled", false);
        SmartBoolean FUNNEL = new SmartBoolean("Enabled Subsystems/Funnel Is Enabled", false);
        SmartBoolean CLIMB = new SmartBoolean("Enabled Subsystems/Climb Is Enabled", false);
        SmartBoolean FROGGY = new SmartBoolean("Enabled Subsystems/Froggy Is Enabled", false);
        SmartBoolean VISION = new SmartBoolean("Enabled Subsystems/Vision Is Enabled", false);
    }

    public interface Swerve {
        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.03;

        public interface Constraints {
            double MAX_MODULE_SPEED = 4.9;
    
            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity (m per s)", 3.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration (m per s^2)", 5.0);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity (rad per s)", Units.degreesToRadians(360));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration (rad per s^2)", Units.degreesToRadians(720));
    
            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());
        }

        public interface Alignment {
            public interface Constraints {
                SmartNumber MAX_VELOCITY = new SmartNumber("Alignment/Constraints/Max Velocity (m per s)", 2.0);
                SmartNumber MAX_ACCELERATION = new SmartNumber("Alignment/Constraints/Max Acceleration (m per s^2)", 3.0);
                SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Alignment/Constraints/Max Angular Velocity (rad per s)", Units.degreesToRadians(360));
                SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Alignment/Constraints/Max Angular Acceleration (rad per s^2)", Units.degreesToRadians(720));
            }

            public interface Tolerances {
                SmartNumber X_TOLERANCE = new SmartNumber("Alignment/Tolerances/X Tolerance (m)", 0.02); 
                SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Tolerances/Y Tolerance (m)", 0.02);
                SmartNumber THETA_TOLERANCE = new SmartNumber("Alignment/Tolerances/Theta Tolerance (rad)", Units.degreesToRadians(4));
    
                SmartNumber MAX_VELOCITY_WHEN_ALIGNED = new SmartNumber("Alignment/Tolerances/Max Velocity When Aligned", 0.15);

                double ALIGNMENT_DEBOUNCE = 0.15;
            }

            public interface Targets {
                double TARGET_DISTANCE_FROM_REEF_L2_BACK = 0.0;
                double TARGET_DISTANCE_FROM_REEF_L2_FRONT = 0.0;
                double TARGET_DISTANCE_FROM_REEF_L3_BACK = 0.0;
                double TARGET_DISTANCE_FROM_REEF_L3_FRONT = 0.0;
                double TARGET_DISTANCE_FROM_REEF_L4_FRONT = 0.0;
                double TARGET_DISTANCE_FROM_REEF_L4_BACK = 0.0;

                double TARGET_DISTANCE_FROM_ALGAE_L2 = 0.0;
                double TARGET_DISTANCE_FROM_ALGAE_L3 = 0.0;

                double TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE = 1.0;
            }
        }
    }

    public interface Vision {
        Vector<N3> MIN_STDDEVS = VecBuilder.fill(0.3, 0.3, 5);
    }

    public interface Shooter {
        SmartNumber CORAL_SHOOT_SPEED_FORWARD = new SmartNumber("Coral Shoot Speed Forward", 0.75);
        SmartNumber CORAL_SHOOT_SPEED_REVERSE = new SmartNumber("Coral Shoot Speed Reverse", -0.75);
        SmartNumber CORAL_ACQUIRE_SPEED = new SmartNumber("Coral Acquire Speed", 0.3);
        SmartNumber ALGAE_ACQUIRE_SPEED = new SmartNumber("Algae Acquire Speed", -0.45);
        SmartNumber ALGAE_SHOOT_SPEED = new SmartNumber("Algae Shoot Speed", 0.45);

        double HAS_CORAL_DEBOUNCE = 0.2;

        double STALL_DETECTION_DEBOUNCE = 0.5;
        double STALL_CURRENT_THRESHOLD = 30;
    }

    public interface Funnel {
        SmartNumber FORWARD_SPEED = new SmartNumber("Funnel/Forward Speed", 0.4);
        SmartNumber REVERSE_SPEED = new SmartNumber("Funnel/Reverse Speed", -0.4);

        double STALL_CURRENT = 30;
        double STALL_DETECTION_TIME = 0.25;
        double MIN_REVERSE_TIME = 1.0;

        double HAS_CORAL_DEBOUNCE = 0.5;
    }

    public interface Elevator {

        double MAX_VELOCITY_METERS_PER_SECOND = 1.0;
        double MAX_ACCEL_METERS_PER_SECOND_PER_SECOND = 1.0;

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
        Rotation2d L2_ANGLE_FRONT = Rotation2d.fromDegrees(-83.385);
        Rotation2d L3_ANGLE_FRONT = Rotation2d.fromDegrees(60.6);
        Rotation2d L4_ANGLE_FRONT = Rotation2d.fromDegrees(75.0);

        Rotation2d L2_ANGLE_BACK = Rotation2d.fromDegrees(171.5);
        Rotation2d L3_ANGLE_BACK = Rotation2d.fromDegrees(146.1);
        Rotation2d L4_ANGLE_BACK = Rotation2d.fromDegrees(150.0);

        Rotation2d ALGAE_L2_ANGLE = Rotation2d.fromDegrees(7);
        Rotation2d ALGAE_L3_ANGLE = Rotation2d.fromDegrees(9);
        Rotation2d BARGE_ANGLE = Rotation2d.fromDegrees(60.0);

        Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(-90);
        Rotation2d FEED_ANGLE = Rotation2d.fromDegrees(-93);

        Rotation2d MAX_VEL = Rotation2d.fromDegrees(150.0);
        Rotation2d MAX_ACCEL = Rotation2d.fromDegrees(200.0);
        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3.0);
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

    public interface Driver {
        double BUZZ_TIME = 1.0;
        double BUZZ_INTENSITY = 1.0;

        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.Constraints.MAX_VELOCITY.get());
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", Swerve.Constraints.MAX_ACCELERATION.get());
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURN_SPEED = new SmartNumber("Driver Settings/Turn/Max Turn Speed (rad per s)", Swerve.Constraints.MAX_ANGULAR_VELOCITY.get());
            SmartNumber MAX_TELEOP_TURN_ACCEL = new SmartNumber("Driver Settings/Turn/Max Turn Accel (rad per s^2)", Swerve.Constraints.MAX_ANGULAR_ACCELERATION.get());
        }
    }

    public interface Operator {
        public interface Froggy {
            Rotation2d MANUAL_ROTATION_VELOCITY = Rotation2d.fromDegrees(10);
        }
        
        public interface Climb {
            double CLIMB_UP_VOLTAGE = 0.0; // Claw is coming up, not robot
            double CLIMB_DOWN_VOLTAGE = -0.0; // Claw is going down, not robot
        }

        public interface Elevator {
            double VOLTAGE_OVERRIDE_DEADBAND = 0.1;
            
            double MAX_VOLTAGE_UP = 6.0;
            double MAX_VOLTAGE_DOWN = -3.0;

            double HEIGHT_OFFSET_PER_CLICK = Units.inchesToMeters(2);
        }

        public interface Arm {
            double VOLTAGE_OVERRIDE_DEADBAND = 0.1;

            double MAX_VOLTAGE_UP = 6.0;
            double MAX_VOLTAGE_DOWN = -3.0;

            Rotation2d ANGLE_OFFSET_PER_CLICK = Rotation2d.fromDegrees(5);
        }
    }
}
