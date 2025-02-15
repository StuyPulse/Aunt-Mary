/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.constants;

public interface Gains {

    public interface Elevator {
        public interface PID {
            double kP = 4.0;
            double kI = 0.0;
            double kD = 0.15;
        }

        public interface FF {
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kG = 2.5;
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
