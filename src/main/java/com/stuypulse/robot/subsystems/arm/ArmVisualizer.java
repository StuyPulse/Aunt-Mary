package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmVisualizer {

    private static final ArmVisualizer instance;
    private final Mechanism2d arm2d;

    private final MechanismRoot2d pivot; 
    private final MechanismLigament2d arm;   

    static {
        instance = new ArmVisualizer();
    }

    public static ArmVisualizer getInstance() {
        return instance;
    }

    ArmVisualizer() {
        arm2d = new Mechanism2d(Units.inchesToMeters(66), Units.inchesToMeters(66));

        pivot = arm2d.getRoot(
            "Pivot",
            Units.inchesToMeters(33),
            Units.inchesToMeters(33)
        );

        arm = new MechanismLigament2d(
            "Arm", 
            Units.inchesToMeters(33), 
            0,
            10, 
            new Color8Bit(Color.kOrange)
        );

        pivot.append(arm);

        SmartDashboard.putData("Visualizers/Arm", arm2d);
    }

    public void update() { 
        arm.setAngle(Arm.getInstance().getCurrentAngle());
    }
}