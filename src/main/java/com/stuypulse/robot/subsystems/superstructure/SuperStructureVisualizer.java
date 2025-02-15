package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SuperStructureVisualizer {
    private static final SuperStructureVisualizer instance;

    static {
        instance = new SuperStructureVisualizer();
    }

    public static SuperStructureVisualizer getInstance() {
        return instance;
    }

    private final Mechanism2d superStructure;

    private final MechanismRoot2d elevatorFixedStageBottomFront;
    private final MechanismRoot2d elevatorFixedStageBottomBack;

    private final MechanismRoot2d elevatorCarriageTopFront;
    private final MechanismRoot2d elevatorCarriageTopBack;

    private final MechanismRoot2d armPivot;
    private final MechanismLigament2d arm;

    private double width;
    private double height;

    private SuperStructureVisualizer() {
        width = Constants.Arm.ARM_LENGTH * 2 + Units.inchesToMeters(3);
        height = Constants.Elevator.MAX_HEIGHT_METERS + Constants.Arm.ARM_LENGTH + Units.inchesToMeters(3);

        superStructure = new Mechanism2d(width, height);

        // FIXED STAGE

        elevatorFixedStageBottomFront = superStructure.getRoot(
            "Elevator Fixed Stage Bottom Front",
            (width + Constants.Elevator.WIDTH)/ 2,
            0
        );

        elevatorFixedStageBottomBack = superStructure.getRoot(
            "Elevator Fixed Stage Bottom Back",
            (width - Constants.Elevator.WIDTH)/ 2,
            0
        );

        elevatorFixedStageBottomFront.append(new MechanismLigament2d(
            "Front Fixed Stage",
            Constants.Elevator.FIXED_STAGE_MAX_HEIGHT,
            90,
            10,
            new Color8Bit(Color.kOrange)
        ));

        elevatorFixedStageBottomBack.append(new MechanismLigament2d(
            "Back Fixed Stage",
            Constants.Elevator.FIXED_STAGE_MAX_HEIGHT,
            90,
            10,
            new Color8Bit(Color.kOrange)
        ));

        // CARRIAGE

        elevatorCarriageTopFront = superStructure.getRoot(
            "Elevator Carriage Top Front",
            (width + Constants.Elevator.WIDTH) / 2 - Units.inchesToMeters(2),
            Constants.Elevator.MIN_HEIGHT_METERS
        );

        elevatorCarriageTopBack = superStructure.getRoot(
            "Elevator Carriage Top Back",
            (width - Constants.Elevator.WIDTH) / 2 + Units.inchesToMeters(2),
            Constants.Elevator.MIN_HEIGHT_METERS
        );

        elevatorCarriageTopFront.append(new MechanismLigament2d(
            "Front Carriage",
            Constants.Elevator.CARRIAGE_LENGTH,
            -90,
            10,
            new Color8Bit(Color.kOrange)
        ));

        elevatorCarriageTopBack.append(new MechanismLigament2d(
            "Back Carriage",
            Constants.Elevator.CARRIAGE_LENGTH,
            -90,
            10,
            new Color8Bit(Color.kOrange)
        ));

        elevatorCarriageTopFront.append(new MechanismLigament2d(
            "Top Carriage",
            Constants.Elevator.WIDTH - (2 * Units.inchesToMeters(2)),
            -180,
            10,
            new Color8Bit(Color.kOrange)
        ));

        // ARM
        armPivot = superStructure.getRoot("Arm Pivot", width/2, Constants.Elevator.MIN_HEIGHT_METERS - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR);

        arm = new MechanismLigament2d(
            "Arm",
            Constants.Arm.ARM_LENGTH,
            -90,
            10,
            new Color8Bit(Color.kBlueViolet)
        );

        armPivot.append(arm);
    }

    public void update(double elevatorHeight, Rotation2d armAngle) {
        elevatorCarriageTopFront.setPosition((width + Constants.Elevator.WIDTH) / 2 - Units.inchesToMeters(2), elevatorHeight);
        elevatorCarriageTopBack.setPosition((width - Constants.Elevator.WIDTH) / 2 + Units.inchesToMeters(2), elevatorHeight);

        armPivot.setPosition(width/2, elevatorHeight - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR);
        arm.setAngle(armAngle);

        SmartDashboard.putData("Visualizers/Superstructure", superStructure);
    }
}
