/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

public interface Gains {

    public interface Swerve {
        public interface Alignment {
            // PIDConstants XY = new PIDConstants(1.492, 0, 0.26879);
            // PIDConstants THETA = new PIDConstants(12.988, 0, 0.77717);
            PIDConstants XY = new PIDConstants(3.0, 0, 0.0);
            PIDConstants THETA = new PIDConstants(3.0, 0, 0.1);
        }
    }

    public interface Elevator {
        public interface PID {
            double kP = 17.185;
            double kI = 0.0;
            double kD = 1.4344;
        }

        public interface FF {
            double kS = 0.28562;
            double kV = 2.8012;
            double kA = 0.16141;
            double kG = 0.81505;
        }
    }

    public interface Arm {
        public interface PID {
            double kP = 0.18304;
            double kI = 0.0;
            double kD = 0.010187;
        }

        public interface FF {
            double kS = 0.0076787; 
            double kV = 0.00847;
            double kA = 0.00153;
            
            double EMPTY_kG = 0.0;
            double EMPTY_kA = 0.0;
            double EMPTY_kV = 0.0;
            double EMPTY_kS = 0.0;

            double CORAL_kG = 0.92147;
            double CORAL_kA = 0.0076787;
            double CORAL_kV = 0.00847;
            double CORAL_kS = 0.00153;

            double ALGAE_kG = 0.92147;
            double ALGAE_kA = 0.0;
            double ALGAE_kV = 0.0;
            double ALGAE_kS = 0.0;
    
        }
    }

    public interface Froggy {
        public interface PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }

        public interface FF {
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double CORAL_kG = 0.0;
            double ALGAE_kG = 0.0;
        }
    }

    public interface Climb {
        public interface PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }

        public interface FF {
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kG = 0.0;
        }
    }
}
