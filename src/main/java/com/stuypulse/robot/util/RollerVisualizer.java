package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RollerVisualizer {
    private static RollerVisualizer instance;

    static {
        instance = new RollerVisualizer();
    }

    public static RollerVisualizer getInstance() {
        return instance;
    }

    private final Mechanism2d canvas;

    private final MechanismRoot2d topRollerPivot;
    private final MechanismRoot2d bottomRollerPivot;

    private final MechanismLigament2d topRoller1;
    private final MechanismLigament2d topRoller2;
    private final MechanismLigament2d topRoller3;
    private final MechanismLigament2d topRoller4;
    
    private final MechanismLigament2d bottomRoller1;
    private final MechanismLigament2d bottomRoller2;
    private final MechanismLigament2d bottomRoller3;
    private final MechanismLigament2d bottomRoller4;

    private double width, height;


    private RollerVisualizer() {
        width = 4;
        height = 4;

        canvas = new Mechanism2d(width, height);

        topRollerPivot = canvas.getRoot("Top Pivot", width/2, height/2 + 1);
        bottomRollerPivot = canvas.getRoot("Bottom Pivot", width/2, height/2 - 1);

        topRoller1 = new MechanismLigament2d("Top Roller 1", 0.2, 90, 2, new Color8Bit(Color.kWhite));
        topRoller2 = new MechanismLigament2d("Top Roller 2", 0.2, 0, 2, new Color8Bit(Color.kWhite)); 
        topRoller3 = new MechanismLigament2d("Top Roller 3", 0.2,180, 2, new Color8Bit(Color.kWhite));
        topRoller4 = new MechanismLigament2d("Top Roller 4", 0.2, -90, 2, new Color8Bit(Color.kWhite)); 

        bottomRoller1 = new MechanismLigament2d("Bottom Roller 1", 0.2, -90, 2, new Color8Bit(Color.kWhite));
        bottomRoller2 = new MechanismLigament2d("Bottom Roller 2", 0.2, 0, 2, new Color8Bit(Color.kWhite));
        bottomRoller3 = new MechanismLigament2d("Bottom Roller 3", 0.2, 90, 2, new Color8Bit(Color.kWhite));
        bottomRoller4 = new MechanismLigament2d("Bottom Roller 4", 0.2, 180, 2, new Color8Bit(Color.kWhite));

        topRollerPivot.append(topRoller1);
        topRollerPivot.append(topRoller2);
        topRollerPivot.append(topRoller3);
        topRollerPivot.append(topRoller4);
        
        bottomRollerPivot.append(bottomRoller1);
        bottomRollerPivot.append(bottomRoller2);
        bottomRollerPivot.append(bottomRoller3);
        bottomRollerPivot.append(bottomRoller4);
    }

    public void updateRollerState(boolean sign) {
        topRoller1.setAngle(sign ? topRoller1.getAngle() - 7 : topRoller1.getAngle() + 7);
        topRoller2.setAngle(sign ? topRoller2.getAngle() - 7 : topRoller2.getAngle() + 7);
        topRoller3.setAngle(sign ? topRoller3.getAngle() - 7 : topRoller3.getAngle() + 7);
        topRoller4.setAngle(sign ? topRoller4.getAngle() - 7 : topRoller4.getAngle() + 7);

        bottomRoller1.setAngle(sign ? bottomRoller1.getAngle() + 7 : bottomRoller1.getAngle() - 7);
        bottomRoller2.setAngle(sign ? bottomRoller2.getAngle() + 7 : bottomRoller2.getAngle() - 7);
        bottomRoller3.setAngle(sign ? bottomRoller3.getAngle() + 7 : bottomRoller3.getAngle() - 7);
        bottomRoller4.setAngle(sign ? bottomRoller4.getAngle() + 7 : bottomRoller4.getAngle() - 7);

        SmartDashboard.putData("Visualizers/Roller", canvas);
    }
}
