/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.subsystems.elevator;

import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ElevatorVisualizer {

    private static final ElevatorVisualizer instance;

    static {
        instance = new ElevatorVisualizer();
    }

    private final Mechanism2d elevator2d;

    private final MechanismRoot2d elevatorBL;
    private final MechanismRoot2d elevatorTR;

    private final MechanismRoot2d stageTwoBL;
    private final MechanismRoot2d stageTwoTR;

    public static ElevatorVisualizer getInstance() {
        return instance;
    }

    public ElevatorVisualizer() {

        // Mechanism2d
        elevator2d = new Mechanism2d(Units.inchesToMeters(14), Units.inchesToMeters(150));

        // Stage One
        // Bottom Left Node
        elevatorBL =
                elevator2d.getRoot(
                        "Elevator BL",
                        Units.inchesToMeters(2),
                        Constants.Elevator.MIN_HEIGHT_METERS);

        elevatorBL.append(
                new MechanismLigament2d(
                        "Left Tower",
                        Units.inchesToMeters(39),
                        90,
                        10,
                        new Color8Bit(Color.kOrange)));

        elevatorBL.append(
                new MechanismLigament2d(
                        "Bottom Tower",
                        Units.inchesToMeters(10), // Change
                        0,
                        10,
                        new Color8Bit(Color.kOrange)));

        // Top Right Node
        elevatorTR =
                elevator2d.getRoot(
                        "Elevator TR",
                        Units.inchesToMeters(12),
                        Units.inchesToMeters(39) + Constants.Elevator.MIN_HEIGHT_METERS);

        elevatorTR.append(
                new MechanismLigament2d(
                        "Right Tower",
                        Units.inchesToMeters(39),
                        -90,
                        10,
                        new Color8Bit(Color.kOrange)));

        elevatorTR.append(
                new MechanismLigament2d(
                        "Top Side",
                        Units.inchesToMeters(10), // Change
                        180,
                        10,
                        new Color8Bit(Color.kOrange)));

        // Stage Two
        // Bottom Left Node
        stageTwoBL =
                elevator2d.getRoot(
                        "Outer BL", Units.inchesToMeters(3), Constants.Elevator.MIN_HEIGHT_METERS);

        stageTwoBL.append(
                new MechanismLigament2d(
                        "Left Side",
                        Units.inchesToMeters(35),
                        90,
                        10,
                        new Color8Bit(Color.kYellow)));

        stageTwoBL.append(
                new MechanismLigament2d(
                        "Bottom Side",
                        Units.inchesToMeters(6),
                        0,
                        10,
                        new Color8Bit(Color.kYellow)));

        // Top Right Node
        stageTwoTR =
                elevator2d.getRoot(
                        "Outer TR",
                        Units.inchesToMeters(12),
                        Units.inchesToMeters(47) + Constants.Elevator.MIN_HEIGHT_METERS);

        stageTwoTR.append(
                new MechanismLigament2d(
                        "Right Side",
                        Units.inchesToMeters(35),
                        -90,
                        10,
                        new Color8Bit(Color.kYellow)));

        stageTwoTR.append(
                new MechanismLigament2d(
                        "Top Side",
                        Units.inchesToMeters(6),
                        180,
                        10,
                        new Color8Bit(Color.kYellow)));

        SmartDashboard.putData("Visualizers/Elevator", elevator2d);
    }

    public void update() {
        // Top of Stage Two is target height
        Elevator elevator = Elevator.getInstance();
        stageTwoBL.setPosition(
                Units.inchesToMeters(4), elevator.getCurrentHeight() + Units.inchesToMeters(4));
        stageTwoTR.setPosition(
                Units.inchesToMeters(10), elevator.getCurrentHeight() + Units.inchesToMeters(39));
    }
}
