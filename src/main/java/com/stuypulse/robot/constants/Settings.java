/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.math.geometry.Rotation2d;


/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {
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
}

public interface Settings { 
    double DT = 0.020; // 20ms Differential Time
  
    public interface Shooter {
        SmartNumber CORAL_FRONT_SPEED = new SmartNumber("Coral Target Speed",0.75);
        SmartNumber CORAL_BACK_SPEED = new SmartNumber("Coral Target Speed",0.75);
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
        SmartNumber MAX_VELOCITY_METERS_PER_SECOND = new SmartNumber("Elevator/Max Velocity (m per s)", 1.0);
        SmartNumber MAX_ACCEL_METERS_PER_SECOND_PER_SECOND = new SmartNumber("Elevator/Max Accel (m per s^2)", 2.0);

        // CHANGE
        double HANDOFF_HEIGHT_METERS = 0.1;
        // front and funnel
        double ALT_L2_HEIGHT_METERS = 0.25;
        double ALT_L3_HEIGHT_METERS = 0.5;
        double ALT_L4_HEIGHT_METERS = 0.75;
        double FUNNEL_L2_HEIGHT_METERS = 0.3; // funnel side; should be higher than L2
        double FUNNEL_L3_HEIGHT_METERS = 0.55; // funnel side; should be higher than L3
        double FUNNEL_L4_HEIGHT_METERS = 0.8;
        
        double RAMP_RATE_CURRENT = 1.0;
        double RAMP_RATE_VOLTAGE = 0.1;

        // FIND OUT REAL GEAR RATIO
        double GEAR_RATIO = 1.0/5.0;
        double CURRENT_LIMIT = 5.0;

        // Magic motion, change RPS
        double TARGET_CRUISE_VELOCITY = 0.0; //Rotations Per Second
        double TARGET_ACCELERATION = 0.0; //Rotations Per Second^2
        double TARGET_JERK = 0.0; //Rotations Per Second^3

        SmartNumber HEIGHT_TOLERANCE_METERS = new SmartNumber("Elevator/Height Tolerance (m)", 0.02);
    
        public interface PID {
            //tune
            SmartNumber kP = new SmartNumber("Elevator/Controller/kP",0.0);
            SmartNumber kI = new SmartNumber("Elevator/Controller/kI",0.0);
            SmartNumber kD = new SmartNumber("Elevator/Controller/kD",0.0);
        }

        public interface FF {
            SmartNumber kS = new SmartNumber("Elevator/Controller/kS",0.20506);
            SmartNumber kV = new SmartNumber("Elevator/Controller/kV",3.7672);
            SmartNumber kA = new SmartNumber("Elevator/Controller/kA", 0.27);
            SmartNumber kG = new SmartNumber("Elevator/Controller/kG", 1.37);
        }
        
        public interface Simulation {
            double SCALE_FACTOR = 0.5 + 2.5/77;
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
        double GEAR_RATIO = 0;
        double ARM_OFFSET = 0;
        double ENCODER_OFFSET = 0;
        SmartNumber PID_RAMPING = new SmartNumber("Arm/PID_RAMP",0);
        SmartNumber FF_RAMPING = new SmartNumber("Arm/FF_RAMP", 0);
        SmartNumber CURRENT_RAMPING = new SmartNumber("Arm/CURRENT_RAMP",0);


        public interface MotionMagic{
            double MAX_VEL = 0;
            double MAX_ACCEL = 0;
            double JERK = 0;
        }
      }
      
  }

