package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class RobotVisualizer {
    public static RobotVisualizer instance;

    static {
        instance = new RobotVisualizer();
    }

    public static RobotVisualizer getInstance() {
        return instance;
    }

    private final Mechanism2d canvas;
    private double width, height;

    private final MechanismRoot2d elevatorFixedStageBottomFront;
    private final MechanismRoot2d elevatorFixedStageBottomBack;

    private final MechanismRoot2d elevatorCarriageTopFront;
    private final MechanismRoot2d elevatorCarriageTopBack;

    private double elevatorHeight;

    private final MechanismRoot2d armPivot;
    private final MechanismRoot2d topRollerPivot;
    private final MechanismRoot2d bottomRollerPivot;

    private final MechanismLigament2d arm;

    private final MechanismLigament2d[] topRollers;
    private final MechanismLigament2d[] bottomRollers;

    private RobotVisualizer() {
        width = Constants.Arm.ARM_LENGTH * 2 + Units.inchesToMeters(3);
        height = Constants.Elevator.MAX_HEIGHT_METERS + Constants.Arm.ARM_LENGTH + Units.inchesToMeters(3);

        canvas = new Mechanism2d(width, height);

        /* ARM VISUALIZER */
        armPivot = canvas.getRoot("Arm Pivot", width/2, Constants.Elevator.MIN_HEIGHT_METERS - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR);

        arm = new MechanismLigament2d(
            "Arm",
            Constants.Arm.ARM_LENGTH,
            -90,
            10,
            new Color8Bit(Color.kBlueViolet)
        );

        armPivot.append(arm);

        /* ROLLER VISUALIZER */

        topRollerPivot = canvas.getRoot("Top Roller Pivot", width/2, Constants.Elevator.MIN_HEIGHT_METERS - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR - Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_TOP_ROLLER);
        bottomRollerPivot = canvas.getRoot("Bottom Roller Pivot", width/2, Constants.Elevator.MIN_HEIGHT_METERS - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR - Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_BOTTOM_ROLLER);

        topRollers = new MechanismLigament2d[4];
        bottomRollers = new MechanismLigament2d[4];
        for (int i = 0; i < 4; i++) {
            topRollers[i] = new MechanismLigament2d("Top Roller " + i, 0.05, 90 * i, 2, new Color8Bit(Color.kWhite));
            topRollerPivot.append(topRollers[i]);
            bottomRollers[i] = new MechanismLigament2d("Bottom Roller " + i, 0.05, 90 * i, 2, new Color8Bit(Color.kWhite));
            bottomRollerPivot.append(bottomRollers[i]);
        }

        /* ELEVATOR VISUALIZER */
        elevatorFixedStageBottomFront = canvas.getRoot(
            "Elevator Fixed Stage Bottom Front",
            (width + Constants.Elevator.WIDTH)/ 2,
            0
        );

        elevatorFixedStageBottomBack = canvas.getRoot(
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

        elevatorCarriageTopFront = canvas.getRoot(
            "Elevator Carriage Top Front",
            (width + Constants.Elevator.WIDTH) / 2 - Units.inchesToMeters(2),
            Constants.Elevator.MIN_HEIGHT_METERS
        );

        elevatorCarriageTopBack = canvas.getRoot(
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

        elevatorHeight = Constants.Elevator.MIN_HEIGHT_METERS;
    }

    public void updateArmAngle(Rotation2d armAngle) {
        armPivot.setPosition(width/2, elevatorHeight - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR);
        arm.setAngle(armAngle);

        topRollerPivot.setPosition(width/2 + Math.cos(Units.degreesToRadians(arm.getAngle())) * Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_TOP_ROLLER, elevatorHeight - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR + Math.sin(Units.degreesToRadians(arm.getAngle())) * Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_TOP_ROLLER);
        bottomRollerPivot.setPosition(width/2 + Math.cos(Units.degreesToRadians(arm.getAngle())) * Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_BOTTOM_ROLLER, elevatorHeight - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR + Math.sin(Units.degreesToRadians(arm.getAngle())) * Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_BOTTOM_ROLLER);

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public void updateRollerSpeed(double speed) {
        for (int i = 0; i < 4; i++) {
            topRollers[i].setAngle(topRollers[i].getAngle() + speed * 20);
            bottomRollers[i].setAngle(bottomRollers[i].getAngle() - speed * 20);
        }

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public void updateElevatorHeight(double elevatorHeight) {
        this.elevatorHeight = elevatorHeight;
        elevatorCarriageTopFront.setPosition((width + Constants.Elevator.WIDTH) / 2 - Units.inchesToMeters(2), elevatorHeight);
        elevatorCarriageTopBack.setPosition((width - Constants.Elevator.WIDTH) / 2 + Units.inchesToMeters(2), elevatorHeight);

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

}
