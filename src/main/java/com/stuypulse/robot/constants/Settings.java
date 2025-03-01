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
        double CLEARANCE_DISTANCE_FROM_REEF = Units.inchesToMeters(14.25 + 1); // From bumper
        double CLEARANCE_DISTANCE_FROM_REEF_FROGGY = Units.inchesToMeters(10); // From bumper
        // Rotation2d MIN_ARM_ANGLE_TO_IGNORE_CLEARANCE_FRONT = Rotation2d.fromDegrees(30);
        // Rotation2d MIN_ARM_ANGLE_TO_IGNORE_CLEARANCE_BACK = Rotation2d.fromDegrees(200);
    }

    public interface Swerve {
        double MODULE_VELOCITY_DEADBAND_M_PER_S = 0.1;
        double ROTATIONAL_DEADBAND_RAD_PER_S = 0.1;
        double NUDGE_FORWARD_SPEED_METERS_PER_SECOND = 0.15;
        public interface Constraints {
            double MAX_MODULE_SPEED = 4.9;
    
            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity (m per s)", 4.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration (m per s^2)", 15.0);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity (rad per s)", Units.degreesToRadians(400));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration (rad per s^2)", Units.degreesToRadians(900));
    
            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());
        }

        public interface Alignment {
            public interface Constraints {
                SmartNumber MAX_VELOCITY = new SmartNumber("Alignment/Constraints/Max Velocity (m per s)", 4.0);
                SmartNumber MAX_ACCELERATION = new SmartNumber("Alignment/Constraints/Max Acceleration (m per s^2)", 11.5);

                SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Alignment/Constraints/Max Angular Velocity (rad per s)", Units.degreesToRadians(400));
                SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Alignment/Constraints/Max Angular Acceleration (rad per s^2)", Units.degreesToRadians(900));
            }

            public interface Tolerances {
                SmartNumber X_TOLERANCE = new SmartNumber("Alignment/Tolerances/X Tolerance (m)", Units.inchesToMeters(2.0)); 
                SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Tolerances/Y Tolerance (m)", Units.inchesToMeters(2.0));
                SmartNumber THETA_TOLERANCE = new SmartNumber("Alignment/Tolerances/Theta Tolerance (rad)", Units.degreesToRadians(5));
    
                SmartNumber MAX_VELOCITY_WHEN_ALIGNED = new SmartNumber("Alignment/Tolerances/Max Velocity When Aligned", 0.15);

                double ALIGNMENT_DEBOUNCE = 0.15;
            }

            public interface Targets {
                // DISTANCE FROM REEF TO BUMPER
                double TARGET_DISTANCE_FROM_REEF_L2_FRONT = -0.02;
                double TARGET_DISTANCE_FROM_REEF_L3_FRONT = -0.02;
                double TARGET_DISTANCE_FROM_REEF_L4_FRONT = -0.02;

                double TARGET_DISTANCE_FROM_REEF_L2_BACK = Units.inchesToMeters(6.5);
                double TARGET_DISTANCE_FROM_REEF_L3_BACK = Units.inchesToMeters(5.5);
                double TARGET_DISTANCE_FROM_REEF_L4_BACK = Units.inchesToMeters(9);

                double TARGET_DISTANCE_FROM_ALGAE_L2 = Units.inchesToMeters(0);
                double TARGET_DISTANCE_FROM_ALGAE_L3 = Units.inchesToMeters(0);

                double TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE = 1.0; // 1 meter was better than 0.85 
            }
        }
    }

    public interface Vision {
        Vector<N3> MIN_STDDEVS = VecBuilder.fill(0.3, 0.3, 1.0);
    }

    public interface Shooter {
        SmartNumber CORAL_SHOOT_SPEED_FORWARD = new SmartNumber("Shooter/Target Speeds/Coral Shoot Speed Forward", 0.75);
        SmartNumber CORAL_SHOOT_SPEED_REVERSE = new SmartNumber("Shooter/Target Speeds/Coral Shoot Speed Reverse", -0.75);
        SmartNumber CORAL_ACQUIRE_SPEED = new SmartNumber("Shooter/Target Speeds/Coral Acquire Speed", 0.15);
        SmartNumber ALGAE_ACQUIRE_SPEED = new SmartNumber("Shooter/Target Speeds/Algae Acquire Speed", -1.0);
        SmartNumber ALGAE_SHOOT_SPEED = new SmartNumber("Shooter/Target Speeds/Algae Shoot Speed", 1.0);

        SmartNumber ALGAE_HOLD_SPEED = new SmartNumber("Shooter/Target Speeds/Algae Hold Speed", -0.15);

        double HAS_CORAL_DEBOUNCE = 0.0;
    }

    public interface Funnel {
        SmartNumber FORWARD_SPEED = new SmartNumber("Funnel/Forward Speed", 1.0);
        SmartNumber REVERSE_SPEED = new SmartNumber("Funnel/Reverse Speed", -1.0);

        double STALL_CURRENT = 30;
        double STALL_DETECTION_TIME = 0.25;
        double MIN_REVERSE_TIME = 1.0;

        double HAS_CORAL_DEBOUNCE = 0.5;
    }

    public interface Elevator {

        double MAX_VELOCITY_METERS_PER_SECOND = 1.5;
        double MAX_ACCEL_METERS_PER_SECOND_PER_SECOND = 2.0;

        double FEED_HEIGHT_METERS = 1.13 - Units.inchesToMeters(2);

        // Coral
        double FRONT_L2_HEIGHT_METERS = 1.538086;
        double FRONT_L3_HEIGHT_METERS = 1.036621;
        double FRONT_L4_HEIGHT_METERS = 1.656494;
        
        double BACK_L2_HEIGHT_METERS = 1.037109;
        double BACK_L3_HEIGHT_METERS = 1.037109;
        double BACK_L4_HEIGHT_METERS = 1.7304;

        // Algae
        double BARGE_HEIGHT_METERS = Constants.Elevator.MAX_HEIGHT_METERS;
        double ALGAE_L2_HEIGHT_METERS = 1.205811;
        double ALGAE_L3_HEIGHT_METERS = 1.622803;

        double CLIMB_HEIGHT_METERS = Constants.Elevator.MIN_HEIGHT_METERS + 0.1;
        double UNSTUCK_CORAL_HEIGHT_METERS = Constants.Elevator.MIN_HEIGHT_METERS + Units.inchesToMeters(12.0);

        double HEIGHT_TOLERANCE_METERS = 0.04;
    }

    public interface Arm {
        Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-82); // Angle that arm makes when resting against the funnel
        Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(201);

        Rotation2d L2_ANGLE_FRONT = Rotation2d.fromDegrees(-59.050619);
        Rotation2d L3_ANGLE_FRONT = Rotation2d.fromDegrees(53.058181);
        Rotation2d L4_ANGLE_FRONT = Rotation2d.fromDegrees(55.361328);

        Rotation2d L2_ANGLE_BACK = Rotation2d.fromDegrees(182.513809);
        Rotation2d L3_ANGLE_BACK = Rotation2d.fromDegrees(150.446319);
        Rotation2d L4_ANGLE_BACK = Rotation2d.fromDegrees(150.859437);

        Rotation2d ALGAE_L2_ANGLE = Rotation2d.fromDegrees(-41.489999);
        Rotation2d ALGAE_L3_ANGLE = Rotation2d.fromDegrees(-41.489999);

        Rotation2d PROCESSOR_ANGLE = Rotation2d.fromDegrees(MIN_ANGLE.getDegrees() + 5);

        Rotation2d CATAPULT_READY_ANGLE = Rotation2d.fromDegrees(-40);
        Rotation2d CATAPULT_SHOOT_ANGLE = Rotation2d.fromDegrees(-15);
        Rotation2d CATAPULT_FINAL_ANGLE = Rotation2d.fromDegrees(80);

        Rotation2d FEED_ANGLE = MIN_ANGLE;
        
        Rotation2d CLIMB_ANGLE = Rotation2d.fromDegrees(MAX_ANGLE.getDegrees() - 5);

        Rotation2d UNSTUCK_CORAL_ANGLE = Rotation2d.fromDegrees(MIN_ANGLE.getDegrees() + 20);

        Rotation2d MAX_VEL = Rotation2d.fromDegrees(350.0);
        Rotation2d MAX_ACCEL = Rotation2d.fromDegrees(700.0);
        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3.0);
    }

    public interface Froggy {
        Rotation2d STOW_ANGLE = Rotation2d.fromDegrees(Constants.Froggy.MAXIMUM_ANGLE.getDegrees() - 3);
        Rotation2d ALGAE_GROUND_PICKUP_ANGLE = Rotation2d.fromDegrees(10);
        Rotation2d CORAL_GROUND_PICKUP_ANGLE = Constants.Froggy.MINIMUM_ANGLE;
        Rotation2d GOLF_TEE_ALGAE_PICKUP_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d L1_SCORING_ANGLE = Rotation2d.fromDegrees(40);
        Rotation2d PROCESSOR_SCORE_ANGLE = Rotation2d.fromDegrees(Constants.Froggy.MAXIMUM_ANGLE.getDegrees() - 15);
        Rotation2d CLIMB_ANGLE = Constants.Froggy.MAXIMUM_ANGLE;

        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3.0);

        SmartNumber ALGAE_INTAKE_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Algae Intake Speed", 1.0);
        SmartNumber ALGAE_OUTTAKE_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Algae Outtake Speed", -0.5);
        SmartNumber CORAL_INTAKE_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Coral Intake Speed", -1.0);
        SmartNumber CORAL_OUTTAKE_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Coral Outtake Speed", 0.5);
        SmartNumber HOLD_ALGAE_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Hold Algae Speed", 0.15);
        SmartNumber HOLD_CORAL_SPEED = new SmartNumber("Froggy/Roller/Target Speeds/Hold Coral Speed", -0.15);

        double CORAL_STALL_CURRENT_THRESHOLD = 80.0;
        double ALGAE_STALL_CURRENT_THRESHOLD = 80.0;
        double STALL_DEBOUNCE_TIME = 0.0;

        Rotation2d MAX_VEL = Rotation2d.fromDegrees(300);
        Rotation2d MAX_ACCEL = Rotation2d.fromDegrees(600);
    }

    public interface Climb {
        double DEFAULT_VOLTAGE = 4; // Used for normal movement
        double OPEN_VOLTAGE_LOW = 1; // Used when getting close to the open angle
        double CLIMB_VOLTAGE = 12; // Used when climbing

        Rotation2d OPEN_ANGLE = Rotation2d.fromDegrees(3.0);
        Rotation2d CLOSED_ANGLE = Rotation2d.fromDegrees(165);
        Rotation2d CLIMBED_ANGLE = Rotation2d.fromDegrees(245);
        
        Rotation2d ANGLE_TOLERANCE_FOR_CLOSED = Rotation2d.fromDegrees(8);
    }

    public interface LED {
        LEDPattern HAS_CORAL_COLOR = LEDPattern.solid(Color.kRed);

        LEDPattern ALIGN_COLOR = LEDPattern.solid(Color.kYellow);
        LEDPattern SCORE_COLOR = LEDPattern.solid(Color.kGreen);
        LEDPattern ABORT_COLOR = LEDPattern.solid(Color.kBlue);

        LEDPattern INTAKE_COLOR = LEDPattern.rainbow(255, 255);

        LEDPattern FUNNEL_UNJAM_COLOR = LEDPattern.solid(Color.kBlue);

        LEDPattern CLIMB_OPEN_COLOR = LEDPattern.rainbow(255, 255);

        LEDPattern CLIMBING_COLOR = LEDPattern.solid(Color.kGreen);

        double DESIRED_TAGS_WHEN_DISABLED = 2; // How many tags we wanna see with one cam when disabled
        LEDPattern DISABLED_ALIGNED = LEDPattern.solid(Color.kPurple); // When able to see DESIRED_TAGS_WHEN_DISABLED+ tags with one cam when disabled

        //Side Alignment color
        LEDPattern LEFT_SIDE_COLOR = LEDPattern.solid(Color.kYellow);
        LEDPattern RIGHT_SIDE_COLOR = LEDPattern.solid(Color.kBlue);
    }

    public interface Driver {
        double BUZZ_TIME = 1.0;
        double BUZZ_INTENSITY = 1.0;

        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.08);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.Constraints.MAX_VELOCITY.get());
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", Swerve.Constraints.MAX_ACCELERATION.get());
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.08);

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
  
    public interface Auton {
        double SHOOTER_WAIT_TIME = 0.75;
    }
}
