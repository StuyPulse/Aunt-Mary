
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import com.pathplanner.lib.path.PathConstraints;

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
        SmartBoolean FUNNEL = new SmartBoolean("Enabled Subsystems/Funnel Is Enabled", true);
        SmartBoolean CLIMB = new SmartBoolean("Enabled Subsystems/Climb Is Enabled", true);
        SmartBoolean FROGGY = new SmartBoolean("Enabled Subsystems/Froggy Is Enabled", true);
        SmartBoolean LEDS = new SmartBoolean("Enabled Subsystems/LEDs", true);
        SmartBoolean SHOOTER_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Vision/Shooter Limelight is Enabled", true);
        SmartBoolean FUNNEL_LIMELIGHT = new SmartBoolean("Enabled Subsystems/Vision/Funnel Limelight is Enabled", true);
    }

    public interface Clearances {
        double CLEARANCE_DISTANCE_FROM_CENTERLINE_BARGE_118 = 1.4;
        double CLEARANCE_DISTANCE_FROM_REEF_ARM = Units.inchesToMeters(17.25); // From bumper
        double CLEARANCE_DISTANCE_FROGGY = Units.inchesToMeters(11); // From bumper
    }

    public interface Swerve {
        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;
        double NUDGE_SPEED_METERS_PER_SECOND = 0.15;
        double NUDGE_SPEED_METERS_PER_SECOND_AUTON = 1.4;
        
        public interface Constraints {    
            double MAX_VELOCITY_M_PER_S = 4.0;
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
                double DEFAULT_MAX_VELOCITY = 4.0;
                double DEFAULT_MAX_ACCELERATION = 15.0;
                double DEFUALT_MAX_ANGULAR_VELOCITY = Units.degreesToRadians(400);
                double DEFAULT_MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(900);

                double MAX_VELOCITY_AUTON = 5.85;
                double MAX_ACCELERATION_AUTON = 17;
            }

            public interface Tolerances {
                double X_TOLERANCE = Units.inchesToMeters(2.0); 
                double Y_TOLERANCE = Units.inchesToMeters(2.0);
                Rotation2d THETA_TOLERANCE = Rotation2d.fromDegrees(3);
    
                double MAX_VELOCITY_WHEN_ALIGNED = 0.15;

                double ALIGNMENT_DEBOUNCE = 0.15;
            }

            public interface Targets {
                // DISTANCE FROM REEF TO BUMPER
                double TARGET_DISTANCE_FROM_REEF_L1_SHOOTER = 0.2;
                double TARGET_DISTANCE_FROM_REEF_L2_FRONT = Units.inchesToMeters(3.5);
                double TARGET_DISTANCE_FROM_REEF_L3_FRONT = -0.02;
                double TARGET_DISTANCE_FROM_REEF_L4_FRONT = 0.0;

                double TARGET_DISTANCE_FROM_REEF_L2_BACK = Units.inchesToMeters(6.5);
                double TARGET_DISTANCE_FROM_REEF_L3_BACK = Units.inchesToMeters(5.5);
                double TARGET_DISTANCE_FROM_REEF_L4_BACK = Units.inchesToMeters(7.5);

                double TARGET_DISTANCE_FROM_REEF_L1_FROGGY = Units.inchesToMeters(5);

                double TARGET_DISTANCE_FROM_ALGAE_L2 = Units.inchesToMeters(0);
                double TARGET_DISTANCE_FROM_ALGAE_L3 = Units.inchesToMeters(0);

                double TARGET_DISTANCE_FROM_CENTERLINE_FOR_CATAPULT = 1.2;
                double TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE_118 = 0.43;

                double HORIZONTAL_DISTANCE_FROM_MIDLINE_FOR_BARGE_AUTO = 0.9;

                double TARGET_DISTANCE_FROM_CORAL_STATION = 0.16;
            }
        }
    }

    public interface Vision {
        Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 694694);
    }

    public interface Shooter {
        double CORAL_SHOOT_SPEED_L1 = 0.22;
        double CORAL_SHOOT_SPEED_L2_FRONT = 0.4;
        double CORAL_SHOOT_SPEED_L2_BACK = 0.4;
        double CORAL_SHOOT_SPEED_L3_FRONT = -0.4;
        double CORAL_SHOOT_SPEED_L3_BACK = 0.4;
        double CORAL_AUTON_SHOOT_SPEED_L4_FRONT = -1.0;
        double CORAL_SHOOT_SPEED_L4_FRONT = -1.0;
        double CORAL_SHOOT_SPEED_L4_BACK = 0.7;

        double CORAL_ACQUIRE_SPEED = 0.2;
        double ALGAE_ACQUIRE_SPEED = -1.0;

        double ALGAE_SHOOT_SPEED = 0.5;
        double ALGAE_HOLD_SPEED = -0.25;

        double UNJAMB_CORAL_BACKWARDS_SPEED = -0.3;
        
        double HAS_CORAL_DEBOUNCE = 0.0;
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
        double L1_HEIGHT_METERS = 1.13 - Units.inchesToMeters(2.25);

        double FRONT_L2_HEIGHT_METERS = 1.53586;
        double FRONT_L3_HEIGHT_METERS = 1.056621;
        double FRONT_L4_HEIGHT_METERS = 1.706494;
        double FRONT_L4_AUTON_HEIGHT_METERS = 1.706494;
        
        double BACK_L2_HEIGHT_METERS = 1.037109;
        double BACK_L3_HEIGHT_METERS = 1.037109;
        double BACK_L4_HEIGHT_METERS = 1.7304;

        // Algae
        double CATAPULT_HEIGHT_METERS = Constants.Elevator.MAX_HEIGHT_METERS;
        double ALGAE_L2_HEIGHT_METERS_FRONT = 1.260986;
        double ALGAE_L3_HEIGHT_METERS_FRONT = 1.469482;
        double ALGAE_L2_HEIGHT_METERS_BACK = Constants.Elevator.MIN_HEIGHT_METERS;
        double ALGAE_L3_HEIGHT_METERS_BACK = 1.352051;

        double PROCESSOR_HEIGHT_METERS = Constants.Elevator.MIN_HEIGHT_METERS;
        double CLIMB_HEIGHT_METERS = Constants.Elevator.MIN_HEIGHT_METERS + 0.1;
        double UNSTUCK_CORAL_HEIGHT_METERS = Constants.Elevator.MIN_HEIGHT_METERS + Units.inchesToMeters(12.0);

        public interface Constraints {
            double MAX_VELOCITY_METERS_PER_SECOND_TELEOP = 2;
            double MAX_ACCEL_METERS_PER_SECOND_PER_SECOND_TELEOP = 6;
    
            double MAX_VELOCITY_METERS_PER_SECOND_AUTON = 2;
            double MAX_ACCEL_METERS_PER_SECOND_PER_SECOND_AUTON = 6;
        }

        double HEIGHT_TOLERANCE_METERS = 0.04;
        double HEIGHT_TOLERANCE_TO_SKIP_CLEARANCE = 0.35;
    }

    public interface Arm {
        Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-82); // Angle that arm makes when resting against the funnel
        Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(201);

        Rotation2d AUTON_L4_ANGLE_FRONT = Rotation2d.fromRadians(55.361328); 

        Rotation2d L1_ANGLE_FRONT = Rotation2d.fromDegrees(-35.139599);
        Rotation2d L2_ANGLE_FRONT = Rotation2d.fromDegrees(-59.050619);
        Rotation2d L3_ANGLE_FRONT = Rotation2d.fromDegrees(53.058181);
        Rotation2d L4_ANGLE_FRONT = Rotation2d.fromDegrees(55.361328);

        Rotation2d L2_ANGLE_BACK = Rotation2d.fromDegrees(182.513809);
        Rotation2d L3_ANGLE_BACK = Rotation2d.fromDegrees(150.446319);
        Rotation2d L4_ANGLE_BACK = Rotation2d.fromDegrees(150.859437);

        Rotation2d ALGAE_L2_ANGLE_FRONT = Rotation2d.fromDegrees(-42.391385);
        Rotation2d ALGAE_L3_ANGLE_FRONT = Rotation2d.fromDegrees(-30.579658);

        Rotation2d ALGAE_L2_ANGLE_BACK = Rotation2d.fromDegrees(160.076257);
        Rotation2d ALGAE_L3_ANGLE_BACK = Rotation2d.fromDegrees(149.102399);

        Rotation2d PROCESSOR_ANGLE = Rotation2d.fromDegrees(MIN_ANGLE.getDegrees());

        Rotation2d CATAPULT_READY_ANGLE = Rotation2d.fromDegrees(-45);
        Rotation2d CATAPULT_SHOOT_ANGLE = Rotation2d.fromDegrees(-20);
        Rotation2d CATAPULT_FINAL_ANGLE = Rotation2d.fromDegrees(80);

        Rotation2d FEED_ANGLE = Rotation2d.fromDegrees(-81);
        
        Rotation2d CLIMB_ANGLE = Rotation2d.fromDegrees(MAX_ANGLE.getDegrees() - 5);

        Rotation2d UNSTUCK_CORAL_ANGLE = Rotation2d.fromDegrees(MIN_ANGLE.getDegrees() + 20);

        public interface Constraints {
            Rotation2d MAX_VEL_TELEOP = Rotation2d.fromDegrees(400.0);
            Rotation2d MAX_ACCEL_TELEOP = Rotation2d.fromDegrees(800.0);

            Rotation2d MAX_VEL_BACK_TO_FEED = Rotation2d.fromDegrees(200.0);
            Rotation2d MAX_ACCEL_BACK_TO_FEED = Rotation2d.fromDegrees(600.0);
    
            Rotation2d MAX_VEL_AUTON = Rotation2d.fromDegrees(350.0);
            Rotation2d MAX_ACCEL_AUTON = Rotation2d.fromDegrees(700.0);

            Rotation2d MAX_VEL_CATAPULT = Rotation2d.fromDegrees(350.0);
            Rotation2d MAX_ACCEL_CATAPULT = Rotation2d.fromDegrees(700.0);
        }

        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(5.0);
        Rotation2d ANGLE_TOLERANCE_TO_SKIP_CLEARANCE = Rotation2d.fromDegrees(15.0);
    }

    public interface Froggy {
        Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(Constants.Froggy.MAXIMUM_ANGLE.getDegrees() - 9);
        Rotation2d ALGAE_GROUND_PICKUP_ANGLE = Rotation2d.fromDegrees(20 - 5);
        Rotation2d CORAL_GROUND_PICKUP_ANGLE = Constants.Froggy.MINIMUM_ANGLE;
        Rotation2d L1_SCORING_ANGLE = Rotation2d.fromDegrees(47);
        Rotation2d PROCESSOR_SCORE_ANGLE = Rotation2d.fromDegrees(Constants.Froggy.MAXIMUM_ANGLE.getDegrees() - 15);
        Rotation2d CLIMB_ANGLE = Constants.Froggy.MAXIMUM_ANGLE;

        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3.0);

        SmartNumber ALGAE_INTAKE_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Algae Intake Speed", 1.0);
        SmartNumber ALGAE_OUTTAKE_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Algae Outtake Speed", -0.5);
        SmartNumber CORAL_INTAKE_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Coral Intake Speed", -1.0);
        SmartNumber CORAL_OUTTAKE_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Coral Outtake Speed", 0.2);
        SmartNumber HOLD_ALGAE_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Hold Algae Speed", 0.3);
        SmartNumber HOLD_CORAL_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Hold Coral Speed", -0.15);

        Rotation2d MAX_VEL = Rotation2d.fromDegrees(500);
        Rotation2d MAX_ACCEL = Rotation2d.fromDegrees(1000);
    }

    public interface Climb {
        double DEFAULT_VOLTAGE = 4; // Used for normal movement
        double OPEN_VOLTAGE_LOW = 1; // Used when getting close to the open angle
        double CLIMB_VOLTAGE = 12; // Used when climbing

        Rotation2d OPEN_ANGLE = Rotation2d.fromDegrees(3.0);
        Rotation2d CLOSED_ANGLE = Rotation2d.fromDegrees(165);
        Rotation2d CLIMBED_ANGLE = Rotation2d.fromDegrees(245);
        Rotation2d SHIMMY_ANGLE = Rotation2d.fromDegrees(70);
        
        Rotation2d ANGLE_TOLERANCE_FOR_CLOSED_AND_SHIMMY = Rotation2d.fromDegrees(8);
    }

    public interface LED {
        LEDPattern HAS_CORAL_COLOR = LEDPattern.solid(Color.kBlue);
        LEDPattern CORAL_STATION_ALIGN_COLOR = LEDPattern.solid(Color.kRed);

        LEDPattern MANUAL_SHOOT_COLOR = LEDPattern.solid(Color.kWhite);

        LEDPattern DEFAULT_ALIGN_COLOR = LEDPattern.solid(Color.kYellow);
        LEDPattern ALIGN_RIGHT_COLOR = LEDPattern.solid(Color.kRed);
        LEDPattern SCORE_COLOR = LEDPattern.solid(Color.kGreen);

        LEDPattern PROCESSOR_SCORE_ANGLE = LEDPattern.solid(Color.kPurple);
        LEDPattern INTAKE_COLOR_ALGAE = LEDPattern.solid(Color.kGreen);
        LEDPattern FROGGY_INTAKE_COLOR_CORAL = LEDPattern.solid(Color.kWhite);
        
        LEDPattern FUNNEL_UNJAM_COLOR = LEDPattern.solid(Color.kBlue);

        LEDPattern CLIMB_OPEN_COLOR = LEDPattern.solid(Color.kYellow);

        LEDPattern CLIMBING_COLOR = LEDPattern.solid(Color.kGreen);

        LEDPattern SHIMMY_COLOR = LEDPattern.solid(Color.kRed);

        LEDPattern AUTON_TO_REEF_COLOR = LEDPattern.solid(Color.kPurple);
        LEDPattern AUTON_TO_HP_COLOR = LEDPattern.solid(Color.kRed);

        double DESIRED_TAGS_WHEN_DISABLED = 2; // How many tags we wanna see with one cam when disabled
        LEDPattern DISABLED_ALIGNED = LEDPattern.solid(Color.kPurple); // When able to see DESIRED_TAGS_WHEN_DISABLED+ tags with one cam when disabled
    }

    public interface Driver {
        double BUZZ_TIME = 1.0;
        double BUZZ_INTENSITY = 1.0;

        double BRANCH_OVERRIDE_DEADBAND = 0.15;

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
