
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
            // sim gains
            double kP = 3.5;
            double kI = 0;
            double kD = 0.2;
            double akP = 3.0;
            double akI = 0;
            double akD = 0.1;

            PIDConstants XY = new PIDConstants(3.5, 0, 0.2); // real robot gains
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
                double kP = 95.308; //0.4 * 360; // 95.308;
                double kI = 0.0 * 360;

                double kD = 10.912;// 0.032 * 360;
            }

            public interface FF {
                double kS = 0.050592; // .2272
                double kV = 3.4737; // 0.0095901 * 360;
                double kA = 0.38111; // 0.0015361 * 360;
                double kG = 0.80784;// 0.69269;
            }
        }

        public interface AlgaeCatapult  {
            public interface PID {
                double kP = 0.15744 * 360;
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
                double kP = 0.080945 * 360; //57.424
                double kI = 0.0 * 360;
                double kD = 0.00078716 * 360; //6.7423
            }

            public interface FF {
                double kS = 0.078337; // 0.14148
                double kV = 0.010338 * 360; // 3.5003
                double kA = 0.001046 * 360; // 0.41248
                double kG = 0.3; // 0.42645
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
