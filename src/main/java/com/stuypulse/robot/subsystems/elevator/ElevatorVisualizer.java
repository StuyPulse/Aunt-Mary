package com.stuypulse.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorVisualizer {

    private static final ElevatorVisualizer instance;

    static {
        instance = new ElevatorVisualizer();
    }

    private final Mechanism2d elevator2d;

    private final MechanismRoot2d elevatorBL;
    private final MechanismRoot2d elevatorTR;

    private final MechanismRoot2d outerBL;
    private final MechanismRoot2d outerTR;

    private final MechanismRoot2d innerBL;
    private final MechanismRoot2d innerTR;

    public static ElevatorVisualizer getInstance(){
        return instance;
    } 

    public ElevatorVisualizer() {

        // Mechanism2d
        elevator2d = new Mechanism2d(Units.inchesToMeters(17), Units.inchesToMeters(150));
        
        // Stage One
        // Bottom Left Node 
        elevatorBL = elevator2d.getRoot("Elevator BL", Units.inchesToMeters(2), Constants.Elevator.MIN_HEIGHT_METERS);

        elevatorBL.append(new MechanismLigament2d(
            "Left Tower",
            Units.inchesToMeters(47), 
            90,
            10, 
            new Color8Bit(Color.kOrange)
            )
        );

        elevatorBL.append(new MechanismLigament2d(
            "Bottom Tower",
            Units.inchesToMeters(11), 
            0,
            10, 
            new Color8Bit(Color.kOrange)
            )
        );

        // Top Right Node
        elevatorTR = elevator2d.getRoot("Elevator TR", Units.inchesToMeters(13), Units.inchesToMeters(47) + Constants.Elevator.MIN_HEIGHT_METERS);

        elevatorTR.append(new MechanismLigament2d(
            "Right Tower", 
            Units.inchesToMeters(47),
            -90,
            10,
            new Color8Bit(Color.kOrange)
            )
        );

        elevatorTR.append(new MechanismLigament2d(
            "Top Side",
            Units.inchesToMeters(11),
            180, 
            10, 
            new Color8Bit(Color.kOrange)
            )
        );

        // Stage Two
        // Bottom Left Node
        outerBL = elevator2d.getRoot("Outer BL", Units.inchesToMeters(3), Constants.Elevator.MIN_HEIGHT_METERS);

        outerBL.append(new MechanismLigament2d(
            "Left Side",
            Units.inchesToMeters(47),
            90, 
            10,
            new Color8Bit(Color.kYellow)
            )
        );
    
        outerBL.append(new MechanismLigament2d(
            "Bottom Side",
            Units.inchesToMeters(9),
            0,
            10,
            new Color8Bit(Color.kYellow)
            )
        );
        
        // Top Right Node
        outerTR = elevator2d.getRoot("Outer TR", Units.inchesToMeters(12), Units.inchesToMeters(47) + Constants.Elevator.MIN_HEIGHT_METERS);

        outerTR.append(new MechanismLigament2d(
            "Top Side",
            Units.inchesToMeters(9),
            180,
            10,
            new Color8Bit(Color.kYellow)
            )
        );

        outerTR.append(new MechanismLigament2d(
            "Right Side",
            Units.inchesToMeters(47),
            -90,
            10,
            new Color8Bit(Color.kYellow)
            )
        );
        
        // Carriage
        // Bottom Left Node
        innerBL = elevator2d.getRoot("Inner BL", Units.inchesToMeters(4), Units.inchesToMeters(1) + Constants.Elevator.MIN_HEIGHT_METERS);

        innerBL.append(new MechanismLigament2d(
            "Left Side",
            Units.inchesToMeters(7),
            90, 
            10,
            new Color8Bit(Color.kPink)
            )
        );

        innerBL.append(new MechanismLigament2d(
            "Bottom Side",
            Units.inchesToMeters(7),
            0,
            10,
            new Color8Bit(Color.kPink)
            )
        );

        // Top Right Node
        innerTR = elevator2d.getRoot("Inner TR", Units.inchesToMeters(11), Units.inchesToMeters(8) + Constants.Elevator.MIN_HEIGHT_METERS);

        innerTR.append(new MechanismLigament2d(
            "Top Side",
            Units.inchesToMeters(7),
            180,
            10,
            new Color8Bit(Color.kPink)
            )
        );
            
        innerTR.append(new MechanismLigament2d(
            "Right Side",
            Units.inchesToMeters(7),
            -90, 
            10,
            new Color8Bit(Color.kPink)
            )
        );

        SmartDashboard.putData("Visualizers/Elevator", elevator2d);
    }

    public void update() {
        // Top of Carriage is Target Height
        Elevator elevator = Elevator.getInstance();

        if (elevator.getTargetHeight() <= Units.inchesToMeters(47)) {
            innerBL.setPosition(Units.inchesToMeters(4), elevator.getCurrentHeight() + Units.inchesToMeters(1));
            innerTR.setPosition(Units.inchesToMeters(11), elevator.getCurrentHeight() + Units.inchesToMeters(8));
        } else {
            outerBL.setPosition(Units.inchesToMeters(3), (elevator.getCurrentHeight() - Constants.Elevator.MIN_HEIGHT_METERS));
            outerTR.setPosition(Units.inchesToMeters(12), (elevator.getCurrentHeight() - Constants.Elevator.MIN_HEIGHT_METERS) + Units.inchesToMeters(47));
    
            innerBL.setPosition(Units.inchesToMeters(4), elevator.getCurrentHeight() + Units.inchesToMeters(1));
            innerTR.setPosition(Units.inchesToMeters(11), elevator.getCurrentHeight() + Units.inchesToMeters(8));
        }

    }

}