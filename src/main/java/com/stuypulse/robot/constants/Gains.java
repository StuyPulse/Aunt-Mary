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
            PIDConstants XY = new PIDConstants(1.0, 0, 0.0);
            PIDConstants THETA = new PIDConstants(12.988, 0, 0.77717);
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
            double kP = 0.045;
            double kI = 0.0;
            double kD = 0.008;
        }

        public interface FF {
            double kS = 0.0; 
            double kV = 0.0;
            double kA = 0.0;

            double kG_EMPTY = 2.1;
            double kG_Coral = 2.1; 
            double kG_Algae = 2.1;
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
            double kG = 0.0;
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
