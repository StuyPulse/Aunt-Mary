/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmElevatorVisualizer {

    private static final ArmElevatorVisualizer instance;

    static {
        instance = new ArmElevatorVisualizer();
    }

    // Canvas
    private final Mechanism2d armElevator2d;

    // Elevator
    private final MechanismRoot2d elevatorBL;
    private final MechanismRoot2d elevatorTR;

    private final MechanismRoot2d stageTwoBL;
    private final MechanismRoot2d stageTwoTR;

    // Arm
    private final MechanismRoot2d pivot;
    private final MechanismLigament2d stick;

    // Funnel
    private final MechanismRoot2d funnelBL;
    private final MechanismRoot2d funnelTR;

    public static ArmElevatorVisualizer getInstance() {
        return instance;
    }

    public ArmElevatorVisualizer() {

        // Mechanism2d
        armElevator2d = new Mechanism2d(Units.inchesToMeters(70), Units.inchesToMeters(150));

        // Arm
        pivot = armElevator2d.getRoot(
                "Arm Origin", 
                Units.inchesToMeters(35), 
                Units.inchesToMeters(39));

        stick = new MechanismLigament2d(
            "Stick", 
            Units.inchesToMeters(29), 
            270, 
            10, 
            new Color8Bit(Color.kAqua));

        pivot.append(stick);

        // Stage One
        // Bottom Left Node
        elevatorBL =
                armElevator2d.getRoot(
                        "Elevator BL",
                        Units.inchesToMeters(30),
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
                        Units.inchesToMeters(10),
                        0,
                        10,
                        new Color8Bit(Color.kOrange)));

        // Top Right Node
        elevatorTR =
                armElevator2d.getRoot(
                        "Elevator TR",
                        Units.inchesToMeters(40),
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
                        Units.inchesToMeters(10),
                        180,
                        10,
                        new Color8Bit(Color.kOrange)));

        // Stage Two
        // Bottom Left Node
        stageTwoBL =
                armElevator2d.getRoot(
                        "Outer BL", Units.inchesToMeters(31.5), Constants.Elevator.MIN_HEIGHT_METERS);

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
                        Units.inchesToMeters(7),
                        0,
                        10,
                        new Color8Bit(Color.kYellow)));

        // Top Right Node
        stageTwoTR =
                armElevator2d.getRoot(
                        "Outer TR",
                        Units.inchesToMeters(38.5),
                        Units.inchesToMeters(35) + Constants.Elevator.MIN_HEIGHT_METERS);

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
                        Units.inchesToMeters(7),
                        180,
                        10,
                        new Color8Bit(Color.kYellow)));

        // Funnel
        // Bottom Left Node
        funnelBL =
                armElevator2d.getRoot(
                        "Funnel BL",
                        Units.inchesToMeters(17), 
                        Units.inchesToMeters(23)); 

        funnelBL.append(
                new MechanismLigament2d(
                        "Left Side",
                        Units.inchesToMeters(14),
                        60,
                        10,
                        new Color8Bit(Color.kPurple)));

        funnelBL.append(
                new MechanismLigament2d(
                        "Bottom Side",
                        Units.inchesToMeters(12),
                        -30,
                        10,
                        new Color8Bit(Color.kPurple)));

        // Top Right Node
        funnelTR =
                armElevator2d.getRoot(
                        "Funnel TR",
                        Units.inchesToMeters(29), 
                        Units.inchesToMeters(23)
                        );

        funnelTR.append(
                new MechanismLigament2d(
                        "Right Side",
                        Units.inchesToMeters(5.5),
                        235,
                        10,
                        new Color8Bit(Color.kPurple)));

        funnelTR.append(
                new MechanismLigament2d(
                        "Top Side",
                        Units.inchesToMeters(14), 
                        110,
                        10,
                        new Color8Bit(Color.kPurple)));

        SmartDashboard.putData("Visualizers/Arm-Elevator", armElevator2d);
    }

    public void update() {
        // Top of Stage Two is target height
        Elevator elevator = Elevator.getInstance();
        Arm arm = Arm.getInstance();

        pivot.setPosition(
                Units.inchesToMeters(35),
                elevator.getCurrentHeight() + Units.inchesToMeters(39));

        stick.setAngle(arm.getCurrentAngle());

        stageTwoBL.setPosition(
                Units.inchesToMeters(31.5), elevator.getCurrentHeight() + Units.inchesToMeters(4));

        stageTwoTR.setPosition(
                Units.inchesToMeters(38.5), elevator.getCurrentHeight() + Units.inchesToMeters(39));

        funnelBL.setPosition(
                Units.inchesToMeters(17), Units.inchesToMeters(23) + Constants.Elevator.MIN_HEIGHT_METERS 
                );

        funnelTR.setPosition(
                Units.inchesToMeters(29), Units.inchesToMeters(23) + Constants.Elevator.MIN_HEIGHT_METERS
                );
    }
}
