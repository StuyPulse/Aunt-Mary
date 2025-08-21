
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
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
    boolean DEBUG_MODE = true;
    String CANIVORE_NAME = "CANIVORE";
    
    public interface EnabledSubsystems {
        SmartBoolean SWERVE = new SmartBoolean("Enabled Subsystems/Swerve Is Enabled", true);
        SmartBoolean ARM = new SmartBoolean("Enabled Subsystems/Arm Is Enabled", true);
        SmartBoolean ELEVATOR = new SmartBoolean("Enabled Subsystems/Elevator Is Enabled", true);
        SmartBoolean SHOOTER = new SmartBoolean("Enabled Subsystems/Shooter Is Enabled", true);
        SmartBoolean FUNNEL = new SmartBoolean("Enabled Subsystems/Funnel Is Enabled", false);
        SmartBoolean CLIMB = new SmartBoolean("Enabled Subsystems/Climb Is Enabled", true);
        SmartBoolean FROGGY = new SmartBoolean("Enabled Subsystems/Froggy Is Enabled", true);
        SmartBoolean LEDS = new SmartBoolean("Enabled Subsystems/LEDs", true);
        SmartBoolean SHOOTER_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Vision/Shooter Limelight is Enabled", true);
        SmartBoolean FUNNEL_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Vision/Funnel Limelight is Enabled", true);
        SmartBoolean FROGGY_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Vision/Froggy Limelight", true);
    }

    public interface Clearances {
        double CLEARANCE_DISTANCE_FROM_CENTERLINE_BARGE_118 = 1.4 - Units.feetToMeters(1) + Units.inchesToMeters(6);
        double CLEARANCE_DISTANCE_FROM_REEF_ARM = Units.inchesToMeters(19.25); // From bumper
        double CLEARANCE_DISTANCE_FROGGY = Units.inchesToMeters(14); // From bumper
        double CLEARANCE_DISTANCE_FROM_REEF_ARM_ALGAE = Units.inchesToMeters(0);

        double CLEARANCE_DISTANCE_CORAL_STATION_ALIGN_FUNNEL_SIDE = Units.inchesToMeters(9);
        double CLEARANCE_DISTANCE_CORAL_STATION_ALIGN_FROGGY_SIDE = Units.inchesToMeters(-9);
    }

    public interface Swerve {
        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;
        double NUDGE_SPEED_METERS_PER_SECOND = 0.15;
        double NUDGE_SPEED_METERS_PER_SECOND_AUTON = 1.4;
        
        public interface Constraints {    
            double MAX_VELOCITY_M_PER_S = 4.3;
            double MAX_ACCEL_M_PER_S_SQUARED = 15.0;
            double MAX_ANGULAR_VEL_RAD_PER_S = Units.degreesToRadians(400);
            double MAX_ANGULAR_ACCEL_RAD_PER_S = Units.degreesToRadians(900);
    
            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY_M_PER_S,
                    MAX_ACCEL_M_PER_S_SQUARED,
                    MAX_ANGULAR_VEL_RAD_PER_S,
                    MAX_ANGULAR_ACCEL_RAD_PER_S);
        }

        public interface Alignment {
            public interface Constraints {
                double DEFAULT_MAX_VELOCITY = 4.3;
                double DEFAULT_MAX_ACCELERATION = 15.0;
                double DEFUALT_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(400);
                double DEFAULT_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(900);
            }

            public interface Tolerances {
                double X_TOLERANCE = Units.inchesToMeters(2.0); 
                double Y_TOLERANCE = Units.inchesToMeters(2.0);
                Rotation2d THETA_TOLERANCE = Rotation2d.fromDegrees(2.0);

                Pose2d POSE_TOLERANCE = new Pose2d(Units.inchesToMeters(2.0), Units.inchesToMeters(2.0), Rotation2d.fromDegrees(2.0));

                double X_TOLERANCE_REEF_ALGAE_PICKUP_READY = Units.inchesToMeters(3.0);
                double Y_TOLERANCE_REEF_ALGAE_PICKUP_READY = Units.inchesToMeters(3.0);
                Rotation2d THETA_TOLERANCE_REEF_PICKUP = Rotation2d.fromDegrees(4.0);

                double X_TOLERANCE_FROGGY = Units.inchesToMeters(3.0);
                double Y_TOLERANCE_FROGGY = Units.inchesToMeters(3.0);
                Rotation2d THETA_TOLERANCE_FROGGY = Rotation2d.fromDegrees(5.0);

                double X_TOLERANCE_BARGE = Units.inchesToMeters(4);
                Rotation2d THETA_TOLERANCE_BARGE = Rotation2d.fromDegrees(10.0);

                double MAX_VELOCITY_WHEN_ALIGNED = 0.15;

                double ALIGNMENT_DEBOUNCE = 0.15;
            }

            public interface Targets {
                // DISTANCE FROM REEF TO BUMPER
                double TARGET_DISTANCE_FROM_REEF_L1_SHOOTER_FRONT = Units.inchesToMeters(4);
                double TARGET_DISTANCE_FROM_REEF_L1_SHOOTER_BACK = Units.inchesToMeters(0);
                double TARGET_DISTANCE_FROM_REEF_L2_FRONT = Units.inchesToMeters(2.5);
                double TARGET_DISTANCE_FROM_REEF_L3_FRONT = Units.inchesToMeters(7); // -0.01
                double TARGET_DISTANCE_FROM_REEF_L4_FRONT = Units.inchesToMeters(1.0);

                double TARGET_DISTANCE_FROM_REEF_L2_BACK = Units.inchesToMeters(6.5);
                double TARGET_DISTANCE_FROM_REEF_L3_BACK = Units.inchesToMeters(6.5);
                double TARGET_DISTANCE_FROM_REEF_L4_BACK = Units.inchesToMeters(7.5);

                double TARGET_DISTANCE_FROM_REEF_L1_FROGGY_VERSATILE = Units.inchesToMeters(1);
                double TARGET_DISTANCE_FROM_REEF_L1_FROGGY_ONE = Units.inchesToMeters(8);
                double TARGET_DISTANCE_FROM_REEF_L1_FROGGY_TWO = Units.inchesToMeters(8);
                double TARGET_DISTANCE_FROM_REEF_L1_FROGGY_THREE = Units.inchesToMeters(4);

                double TARGET_DISTANCE_FROM_ALGAE_L2 = Units.inchesToMeters(0);
                double TARGET_DISTANCE_FROM_ALGAE_L3 = Units.inchesToMeters(-1);

                double TARGET_DISTANCE_FROM_CENTERLINE_FOR_CATAPULT = 1.2;
                double TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE_118 = 0.74 - Units.inchesToMeters(6);

                double Y_DISTANCE_FROM_MIDLINE_FOR_BARGE_AUTO_SHORT = 0.85;
                double Y_DISTANCE_FROM_MIDLINE_FOR_BARGE_AUTO_LONG = 1.0;

                double TARGET_DISTANCE_FROM_CORAL_STATION = 0.12;
                // double TARGET_DISTANCE_FROM_CORAL_STATION_LEFT_RIGHT = Units.inchesToMeters(21);
                double TARGET_DISTANCE_FROM_CORAL_STATION_OUT = Units.inchesToMeters(-18);
                double TARGET_DISTANCE_FROM_CORAL_STATION_IN = Units.inchesToMeters(21);

                Rotation2d ANGLE_FROM_HORIZONTAL_FOR_118 = Rotation2d.fromDegrees(0);
                Rotation2d ANGLE_FROM_HORIZONTAL_FOR_118_AUTON = Rotation2d.fromDegrees(0);
            }
        }
    }

    public interface Vision {
        Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694);
    }

    public interface Shooter {
        double CORAL_SHOOT_SPEED_L1_FRONT = 1.0; // 0.22
        double CORAL_SHOOT_SPEED_L1_BACK = -1.0; // -0.75
        double CORAL_SHOOT_SPEED_L2_FRONT = 1.0; // 0.4
        double CORAL_SHOOT_SPEED_L2_BACK = 1.0; // 0.4
        double CORAL_SHOOT_SPEED_L3_FRONT = 1.0; // 0.5
        double CORAL_SHOOT_SPEED_L3_BACK = 1.0; // 0.4
        double CORAL_AUTON_SHOOT_SPEED_L4_FRONT = -1.0; // -0.5
        double CORAL_SHOOT_SPEED_L4_FRONT = -1.0;
        double CORAL_SHOOT_SPEED_L4_BACK = 1.0; // 0.7
        
        double CORAL_SHOOT_TIME_AUTON = 0.2;

        double CORAL_ACQUIRE_SPEED = 0.5; // 0.35
        double ALGAE_ACQUIRE_SPEED = -1.0;

        double ALGAE_SHOOT_SPEED = 1.0; // 0.5
        double ALGAE_HOLD_SPEED = -0.05; // -0.5

        double UNJAM_CORAL_BACKWARDS_SPEED = -0.3;
        
        double HAS_CORAL_DEBOUNCE = 0.0;

        double CORAL_STATOR_CURRENT_THRESHOLD = 17.0;
    }

    public interface Funnel {
        double FORWARD_SPEED = 1.0;
        double REVERSE_SPEED = -1.0;

        double STALL_CURRENT = 19;
        double STALL_DETECTION_TIME = 0.25;

        double MIN_REVERSE_TIME = 1.0;

        double HAS_CORAL_DEBOUNCE = 0.0;
    }

    public interface Elevator {

        double FEED_HEIGHT_METERS = 1.047119;

        // Coral
        double FRONT_L1_HEIGHT_METERS = 1.16;
        double BACK_L1_HEIGHT_METERS = 1.5;

        double FRONT_L2_HEIGHT_METERS = 1.57586;
        double FRONT_L3_HEIGHT_METERS = 1.760498 + Units.inchesToMeters(1); // 1.0566
        double FRONT_L4_HEIGHT_METERS = 1.706494;
        
        double BACK_L2_HEIGHT_METERS = 1.037109 + Units.inchesToMeters(1);
        double BACK_L3_HEIGHT_METERS = 1.077109 + Units.inchesToMeters(6);
        double BACK_L4_HEIGHT_METERS = 1.7304 + Units.inchesToMeters(2);

        // Algae
        double CATAPULT_HEIGHT_METERS = Constants.Elevator.MAX_HEIGHT_METERS;
        double BARGE_118_HEIGHT_METERS = Constants.Elevator.MAX_HEIGHT_METERS;
        
        double ALGAE_L2_HEIGHT_METERS_FRONT = 1.479980 + Units.inchesToMeters(2);
        double ALGAE_L3_HEIGHT_METERS_FRONT = 1.726074 + Units.inchesToMeters(2);
        double ALGAE_L2_HEIGHT_METERS_BACK = Constants.Elevator.MIN_HEIGHT_METERS;
        double ALGAE_L3_HEIGHT_METERS_BACK = 1.352051 - Units.inchesToMeters(2);

        double GOLF_TEE_ALGAE_PICKUP_HEIGHT = Constants.Elevator.MIN_HEIGHT_METERS;
        double GROUND_ALGAE_PICKUP_HEIGHT = Constants.Elevator.MIN_HEIGHT_METERS;

        double PROCESSOR_HEIGHT_METERS = 1.250732 + Units.inchesToMeters(2);
        double CLIMB_HEIGHT_METERS = Constants.Elevator.MIN_HEIGHT_METERS + 0.1;
        double UNSTUCK_CORAL_HEIGHT_METERS = Constants.Elevator.MIN_HEIGHT_METERS + Units.inchesToMeters(12.0);

        public interface Constraints {
            double MAX_VELOCITY_METERS_PER_SECOND_TELEOP = 2;
            double MAX_ACCEL_METERS_PER_SECOND_PER_SECOND_TELEOP = 4;
    
            double MAX_VELOCITY_METERS_PER_SECOND_AUTON = 2;
            double MAX_ACCEL_METERS_PER_SECOND_PER_SECOND_AUTON = 4;
        }

        double HEIGHT_TOLERANCE_METERS = 0.04;
        double HEIGHT_TOLERANCE_TO_SKIP_CLEARANCE = 0.45;
    }

    public interface Arm {
        double MIN_ANGLE_DEG = -82.0; // Angle that arm makes when resting against the funnel
        double MAX_ANGLE_DEG = 201.0;

        double L1_ANGLE_FRONT_DEG = -35.139599;
        double L2_ANGLE_FRONT_DEG = -59.050619;
        double L3_ANGLE_FRONT_DEG = -38.330078; //53.05
        double L4_ANGLE_FRONT_DEG = 55.361328;

        double L1_ANGLE_BACK_DEG = 150.139599;
        double L2_ANGLE_BACK_DEG = 174.513809;
        double L3_ANGLE_BACK_DEG = 156.446319;
        double L4_ANGLE_BACK_DEG = 150.859437;

        double AUTON_END_DEG = 94.570312;

        double ALGAE_L2_ANGLE_FRONT_DEG = -47.724609;
        double ALGAE_L3_ANGLE_FRONT_DEG = -30.013672; 

        double ALGAE_L2_ANGLE_BACK_DEG = 166.552734; // 160.076257 new setting 5/30/25
        double ALGAE_L3_ANGLE_BACK_DEG = 155.102399;

        double PROCESSOR_ANGLE_DEG = -71.464844;

        // TODO: This angle is going to get wrapped - should be fixed at some point with field testing 
        double GOLF_TEE_ALGAE_PICKUP_ANGLE_DEG = 197.337891;  // -42.636719 + 6
        double GROUND_ALGAE_PICKUP_ANGLE_DEG = -56.347656; // MADE UP, FIND THIS

        double CATAPULT_READY_ANGLE_DEG = -60.0;
        double CATAPULT_SHOOT_ANGLE_DEG = -55.0;
        double CATAPULT_FINAL_ANGLE_DEG = 70.0;

        double BARGE_118_ANGLE_DEG = 90.0;
        double BARGE_SAFE_118_DEG = 60.0; // 80

        double FEED_ANGLE_DEG = -81.0;
        
        double CLIMB_ANGLE_DEG = MAX_ANGLE_DEG - 5.0;

        double UNSTUCK_CORAL_ANGLE_DEG = MIN_ANGLE_DEG + 20.0;

        public interface Constraints {
            // Rotation2d MAX_VEL_TELEOP = Rotation2d.fromDegrees(600.0); 
            // Rotation2d MAX_ACCEL_TELEOP = Rotation2d.fromDegrees(1200.0); 
            // Rotation2d MAX_VEL_TELEOP = Rotation2d.fromRotations(5/3); 
            // Rotation2d MAX_ACCEL_TELEOP = Rotation2d.fromRotations(10/3); 
            double MAX_VEL_TELEOP_DEG = 500.0;
            double MAX_ACCEL_TELEOP_DEG = 1200.0;

            double MAX_VEL_TELEOP_FUNNEL_SIDE = 600.0; // 550
            double MAX_ACCEL_TELEOP_FUNNEL_SIDE = 600.0; // 550


            double MAX_VEL_AUTON = 1200.0;
            double MAX_ACCEL_AUTON = 2400.0;

            double ALGAE_VEL_AUTON = 600.0;
            double ALGAE_ACCEL_AUTON = 800.0;

            double DEFAULT_MAX_VEL_BACK_TO_FEED = 250.0;
            double DEFAULT_MAX_ACCEL_BACK_TO_FEED = 600.0;

            double MAX_VEL_BACK_TO_FEED_AND_PROCESSOR_WITH_ALGAE = 200.0;
            double MAX_ACCEL_BACK_TO_FEED_AND_PROCESSOR_WITH_ALGAE = 500.0;

            double MAX_VEL_CATAPULT = 720.0;
            double MAX_ACCEL_CATAPULT = 1500.0;

            // Rotation2d MAX_VEL_TELEOP_FUNNEL_SIDE = Rotation2d.fromDegrees(600.0); // 550
            // Rotation2d MAX_ACCEL_TELEOP_FUNNEL_SIDE = Rotation2d.fromDegrees(600.0); // 550


            // Rotation2d MAX_VEL_AUTON = Rotation2d.fromDegrees(1200.0);
            // Rotation2d MAX_ACCEL_AUTON = Rotation2d.fromDegrees(2400.0);

            // Rotation2d ALGAE_VEL_AUTON = Rotation2d.fromDegrees(600.0);
            // Rotation2d ALGAE_ACCEL_AUTON = Rotation2d.fromDegrees(800.0);

            // Rotation2d DEFAULT_MAX_VEL_BACK_TO_FEED = Rotation2d.fromDegrees(250.0);
            // Rotation2d DEFAULT_MAX_ACCEL_BACK_TO_FEED = Rotation2d.fromDegrees(600.0);

            // Rotation2d MAX_VEL_BACK_TO_FEED_AND_PROCESSOR_WITH_ALGAE = Rotation2d.fromDegrees(200.0);
            // Rotation2d MAX_ACCEL_BACK_TO_FEED_AND_PROCESSOR_WITH_ALGAE = Rotation2d.fromDegrees(500.0);

            // Rotation2d MAX_VEL_CATAPULT = Rotation2d.fromDegrees(720.0);
            // Rotation2d MAX_ACCEL_CATAPULT = Rotation2d.fromDegrees(1500.0);
        }

        Rotation2d ANGLE_TOLERANCE_FRONT = Rotation2d.fromDegrees(7.0);
        Rotation2d ANGLE_TOLERANCE_BACK = Rotation2d.fromDegrees(7.0);
        Rotation2d ANGLE_TOLERANCE_TO_SKIP_CLEARANCE = Rotation2d.fromDegrees(25.0);
    }

    public interface Froggy {
        Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(Constants.Froggy.MAXIMUM_ANGLE.getDegrees() - 9);
        Rotation2d ALGAE_GROUND_PICKUP_ANGLE = Rotation2d.fromDegrees(20);
        Rotation2d CORAL_GROUND_PICKUP_ANGLE = Constants.Froggy.MINIMUM_ANGLE;
        Rotation2d L1_SCORING_ANGLE_VERSATILE = Rotation2d.fromDegrees(52);
        Rotation2d L1_SCORING_ANGLE_ONE = Rotation2d.fromDegrees(52);
        Rotation2d L1_SCORING_ANGLE_TWO = Rotation2d.fromDegrees(54.667969);
        Rotation2d L1_SCORING_ANGLE_THREE = Rotation2d.fromDegrees(54.667969);
        Rotation2d GOLF_TEE_ALGAE_PICKUP_ANGLE = STOW_ANGLE;
        Rotation2d CLIMB_ANGLE = Constants.Froggy.MAXIMUM_ANGLE;

        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(5.0);

        double ALGAE_INTAKE_SPEED = 1.0;
        double ALGAE_OUTTAKE_SPEED = -0.5;
        double ALGAE_OUTTAKE_SPEED_VERSATILE = -0.5;
        double CORAL_OUTTAKE_SPEED_ONE = 0.2;
        double CORAL_OUTTAKE_SPEED_TWO = 0.3;
        double CORAL_OUTTAKE_SPEED_THREE = 0.2;
        double CORAL_INTAKE_SPEED = -1.0;
        double CORAL_OUTTAKE_SPEED = 0.2;
        double HOLD_ALGAE_SPEED = 0.3;
        double HOLD_CORAL_SPEED = -0.15;

        // Rotation2d MAX_VEL = Rotation2d.fromDegrees(500);
        // Rotation2d MAX_ACCEL = Rotation2d.fromDegrees(1000);
        double MAX_VEL_DEG = 500.0;
        double MAX_ACCEL_DEG = 1000.0;
    }

    public interface Climb {
        double DEFAULT_VOLTAGE = 4; // Used for normal movement
        double OPEN_VOLTAGE_LOW = 1; // Used when getting close to the open angle
        double CLIMB_VOLTAGE = 12; // Used when climbing 12

        double OPEN_ANGLE_DEG = 1.0;
        double CLOSED_ANGLE_DEG = 165;
        double CLIMBED_ANGLE_DEG = 230; // 245 - 3.5 // 230 // 215
        double SHIMMY_ANGLE_DEG = 70;
        
        double ANGLE_TOLERANCE_FOR_CLOSED_AND_SHIMMY_DEG = 7;
    }

    public interface LED {
        LEDPattern HAS_CORAL_COLOR = LEDPattern.solid(Color.kBlue);
        LEDPattern CORAL_STATION_ALIGN_COLOR = LEDPattern.solid(Color.kRed);
        LEDPattern CORAL_STATION_ALIGN_COLOR_LEFT = LEDPattern.solid(Color.kYellow);
        LEDPattern CORAL_STATION_ALIGN_COLOR_RIGHT = LEDPattern.solid(Color.kRed);

        LEDPattern MANUAL_SHOOT_COLOR = LEDPattern.solid(Color.kWhite);

        LEDPattern DEFAULT_ALIGN_COLOR = LEDPattern.solid(Color.kYellow);
        LEDPattern ALIGN_RIGHT_COLOR = LEDPattern.solid(Color.kRed);
        LEDPattern SCORE_COLOR = LEDPattern.solid(Color.kGreen);

        LEDPattern PROCESSOR_SCORE_ANGLE = LEDPattern.solid(Color.kPurple);
        LEDPattern INTAKE_COLOR_ALGAE = LEDPattern.solid(Color.kGreen);
        LEDPattern FROGGY_INTAKE_COLOR_CORAL = LEDPattern.solid(Color.kRed);

        LEDPattern FROGGY_SCORE_ONE = LEDPattern.solid(Color.kBlue);
        LEDPattern FROGGY_SCORE_TWO = LEDPattern.solid(Color.kYellow);
        LEDPattern FROGGY_SCORE_THREE = LEDPattern.solid(Color.kPurple);
        
        LEDPattern FUNNEL_UNJAM_COLOR = LEDPattern.solid(Color.kBlue);

        LEDPattern CLIMB_OPEN_COLOR = LEDPattern.solid(Color.kYellow);

        LEDPattern CLIMBING_COLOR = LEDPattern.solid(Color.kGreen);

        LEDPattern SHIMMY_COLOR = LEDPattern.solid(Color.kRed);

        LEDPattern AUTON_TO_REEF_COLOR = LEDPattern.solid(Color.kPurple);
        LEDPattern AUTON_TO_HP_COLOR = LEDPattern.solid(Color.kRed);

        LEDPattern BARGE_ALIGNING = LEDPattern.solid(Color.kYellow);
        LEDPattern BARGE_ALIGNMENT_DONE = LEDPattern.solid(Color.kRed);

        double DESIRED_TAGS_WHEN_DISABLED = 2; // How many tags we wanna see with one cam when disabled
        LEDPattern DISABLED_ALIGNED = LEDPattern.solid(Color.kPurple); // When able to see DESIRED_TAGS_WHEN_DISABLED+ tags with one cam when disabled
    }

    public interface Driver {
        double BUZZ_TIME = 1.0;
        double BUZZ_INTENSITY = 1.0;

        double BRANCH_OVERRIDE_DEADBAND = 0.15;
        double CORAL_STATION_OVERRIDE_DEADBAND = 0.15; // 0.2

        public interface Drive {
            double DEADBAND = 0.08;

            double RC = 0.05;
            double POWER = 2;

            double MAX_TELEOP_SPEED = Swerve.Constraints.MAX_VELOCITY_M_PER_S;
            double MAX_TELEOP_ACCEL = Swerve.Constraints.MAX_ACCEL_M_PER_S_SQUARED;

            double MAX_TELEOP_SPEED_WHILE_CLIMBING = MAX_TELEOP_SPEED / 2;
        }

        public interface Turn {
            double DEADBAND = 0.08;

            double RC = 0.05;
            double POWER = 2;

            double MAX_TELEOP_TURN_SPEED = Swerve.Constraints.MAX_ANGULAR_VEL_RAD_PER_S;
            double MAX_TELEOP_TURN_ACCEL = Swerve.Constraints.MAX_ANGULAR_ACCEL_RAD_PER_S;

            double MAX_TELEOP_TURN_SPEED_WHILE_CLIMBING = MAX_TELEOP_TURN_SPEED / 2;
        }
    }
}
