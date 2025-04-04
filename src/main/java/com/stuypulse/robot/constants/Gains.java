
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
        public interface CoralAlgae  {
            public interface PID {
                double kP = 0.085744 * 360;
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
            double kP = 0.055824;
            double kI = 0.0;
            double kD = 0.002771;
        }

        public interface FF {
            double kS = 0.17874; 
            double kV = 0.013683;
            double kA = 0.00092838;
            double kG = 0.1976;
        }
    }
}
