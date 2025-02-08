package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.subsystems.elevator.Elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmVisualizer {

    private static final ArmVisualizer instance;

    static {
        instance = new ArmVisualizer();
    }

    private final Mechanism2d arm2d;

    // Elevator

    private final MechanismRoot2d elevatorBL;
    private final MechanismRoot2d elevatorTR;

    private final MechanismRoot2d stageTwoBL;
    private final MechanismRoot2d stageTwoTR;

    // Arm

    private final MechanismRoot2d pivot;

    // Funnel

    private final MechanismRoot2d funnelBL;
    private final MechanismRoot2d funnelTR;

    public static ArmVisualizer getInstance() {
        return instance;
    }

    public ArmVisualizer() {

        // Mechanism2d
        arm2d = new Mechanism2d(Units.inchesToMeters(36), Units.inchesToMeters(150));
        
        // Stage One
        // Bottom Left Node
        elevatorBL =
                arm2d.getRoot(
                        "Elevator BL",
                        Units.inchesToMeters(13),
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
                arm2d.getRoot(
                        "Elevator TR",
                        Units.inchesToMeters(23),
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
                arm2d.getRoot(
                        "Outer BL", Units.inchesToMeters(14), Constants.Elevator.MIN_HEIGHT_METERS);

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
                arm2d.getRoot(
                        "Outer TR",
                        Units.inchesToMeters(22),
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
                        Units.inchesToMeters(6),
                        180,
                        10,
                        new Color8Bit(Color.kYellow)));

        // Arm
        
        pivot = arm2d.getRoot(
                "Arm Origin",
                Units.inchesToMeters(18),
                Units.inchesToMeters(39));

        pivot.append(
                new MechanismLigament2d(
                        "arm",
                        Units.inchesToMeters(20),
                        270,
                        10,
                        new Color8Bit(Color.kAqua)));
                        
        // Funnel 

        // Bottom Left Node
        funnelBL = arm2d.getRoot(
                "Funnel BL",
                Units.inchesToMeters(-15), //Change
                Units.inchesToMeters(22.5)
        );

        funnelBL.append(
                new MechanismLigament2d(
                        "Left Side",
                        Units.inchesToMeters(6.5), //Change
                        60,
                        10,
                        new Color8Bit(Color.kPurple)));

        funnelBL.append(
                new MechanismLigament2d(
                        "Bottom Side",
                        Units.inchesToMeters(12.5),
                        0,
                        10,
                        new Color8Bit(Color.kPurple)));
        
        // Top Right Node
        funnelTR = arm2d.getRoot(
                "Funnel TR",
                Units.inchesToMeters(0.5), //Change
                Units.inchesToMeters(28) //Change
        );
        
        funnelTR.append(
                new MechanismLigament2d(
                        "Right Side",
                        Units.inchesToMeters(6.5), //Change
                        240,
                        10,
                        new Color8Bit(Color.kPurple)));

        funnelTR.append(
                new MechanismLigament2d(
                        "Top Side",
                        Units.inchesToMeters(12.5), //Change
                        180,
                        10,
                        new Color8Bit(Color.kPurple)));

        SmartDashboard.putData("Visualizers/Arm", arm2d);
    }

    public void update() {
        // Top of Stage Two is target height
        Elevator elevator = Elevator.getInstance();
        Arm arm = Arm.getInstance();

        stageTwoBL.setPosition(
                Units.inchesToMeters(14), 
                elevator.getCurrentHeight() + Units.inchesToMeters(4));

        stageTwoTR.setPosition(
                Units.inchesToMeters(22), 
                elevator.getCurrentHeight() + Units.inchesToMeters(39));

        pivot.setPosition(
                Units.inchesToMeters(7),
                elevator.getCurrentHeight() + Units.inchesToMeters(39) // Change
        );

        funnelBL.setPosition(
                Units.inchesToMeters(0), Units.inchesToMeters(22.5) //Change
        );

        funnelBL.setPosition(
                Units.inchesToMeters(12.5), Units.inchesToMeters(30) //Change
        );
    }
}
