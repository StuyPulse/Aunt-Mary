
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
            PIDConstants XY = new PIDConstants(3.5, 0, 0.2);
            PIDConstants THETA = new PIDConstants(3.0, 0, 0.1);
        }
    }

    public interface Elevator {
        public interface PID {
            double kP = 25.669;
            double kI = 0.0;
            double kD = 2.2108;
        }

        public interface FF {
            double kS = 0.27594;
            double kV = 3.0438;
            double kA = 0.10459;
            double kG = 0.67941;
        }
    }

    public interface Arm {
        public interface Coral  {
            public interface PID {
                double kP = 0.4 * 360;
                double kI = 0.0 * 360;
                double kD = 0.032 * 360;
                // double kP = 0.15744 * 360;
                // double kI = 0.0 * 360;
                // double kD = 0.015 * 360;
            }

            public interface FF {
                double kS = 0.2272; 
                double kV = 0.0095901 * 360;
                double kA = 0.0015361 * 360;
                double kG = 0.69269;
            }
        }

        public interface AlgaeCatapult  {
            public interface PID {
                double kP = 0.095744 * 360;
                double kI = 0.0 * 360;
                double kD = 0.015 * 360;
            }

            public interface FF {
                double kS = 0.2272; 
                double kV = 0.0095901 * 360;
                double kA = 0.0015361 * 360;
                double kG = 0.69269;
            }
        }

        public interface Empty  {
            public interface PID {
                double kP = 0.080945 * 360;
                double kI = 0.0 * 360;
                double kD = 0.00078716 * 360;
            }

            public interface FF {
                double kS = 0.078337; 
                double kV = 0.010338 * 360;
                double kA = 0.001046 * 360;
                double kG = 0.3;
            }
        }
    }

    public interface Froggy {
        public interface PID {
            double kP = 0.055824 * 360;
            double kI = 0.0 * 360;
            double kD = 0.002771 * 360;
        }

        public interface FF {
            double kS = 0.17874; 
            double kV = 0.013683 * 360;
            double kA = 0.00092838 * 360;
            double kG = 0.1976;
        }
    }
}
