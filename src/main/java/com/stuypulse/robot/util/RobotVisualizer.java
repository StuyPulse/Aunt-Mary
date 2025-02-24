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
    private final MechanismRoot2d armTopRollerPivot;
    private final MechanismRoot2d armBottomRollerPivot;

    private final MechanismLigament2d arm;

    private final MechanismLigament2d topRoller1;
    private final MechanismLigament2d topRoller2;
    private final MechanismLigament2d topRoller3;
    private final MechanismLigament2d topRoller4;
    
    private final MechanismLigament2d bottomRoller1;
    private final MechanismLigament2d bottomRoller2;
    private final MechanismLigament2d bottomRoller3;
    private final MechanismLigament2d bottomRoller4;

    private RobotVisualizer() {
        width = Constants.Arm.ARM_LENGTH * 2 + Units.inchesToMeters(3);
        height = Constants.Elevator.MAX_HEIGHT_METERS + Constants.Arm.ARM_LENGTH + Units.inchesToMeters(3);

        canvas = new Mechanism2d(width, height);

        /* ARM VISUALIZER */
        armPivot = canvas.getRoot("Arm Pivot", width/2, Constants.Elevator.MIN_HEIGHT_METERS - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR);
        armTopRollerPivot = canvas.getRoot("Top Roller Pivot", width/2, Constants.Elevator.MIN_HEIGHT_METERS - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_ROLLER );
        armBottomRollerPivot = canvas.getRoot("Bottom Roller Pivot", width/2, Constants.Elevator.MIN_HEIGHT_METERS - Constants.Arm.DISTANCE_FROM_PIVOT_TO_BOTTOM_ROLLER);

        arm = new MechanismLigament2d(
            "Arm",
            Constants.Arm.ARM_LENGTH,
            -90,
            10,
            new Color8Bit(Color.kBlueViolet)
        );

        armPivot.append(arm);

        /* ROLLER VISUALIZER */

        topRoller1 = new MechanismLigament2d("Top Roller 1", 0.05, 90, 2, new Color8Bit(Color.kWhite));
        topRoller2 = new MechanismLigament2d("Top Roller 2", 0.05, 0, 2, new Color8Bit(Color.kWhite)); 
        topRoller3 = new MechanismLigament2d("Top Roller 3", 0.05,180, 2, new Color8Bit(Color.kWhite));
        topRoller4 = new MechanismLigament2d("Top Roller 4", 0.05, -90, 2, new Color8Bit(Color.kWhite)); 

        bottomRoller1 = new MechanismLigament2d("Bottom Roller 1", 0.05, -90, 2, new Color8Bit(Color.kWhite));
        bottomRoller2 = new MechanismLigament2d("Bottom Roller 2", 0.05, 0, 2, new Color8Bit(Color.kWhite));
        bottomRoller3 = new MechanismLigament2d("Bottom Roller 3", 0.05, 90, 2, new Color8Bit(Color.kWhite));
        bottomRoller4 = new MechanismLigament2d("Bottom Roller 4", 0.05, 180, 2, new Color8Bit(Color.kWhite));

        armTopRollerPivot.append(topRoller1);
        armTopRollerPivot.append(topRoller2);
        armTopRollerPivot.append(topRoller3);
        armTopRollerPivot.append(topRoller4);
        
        armBottomRollerPivot.append(bottomRoller1);
        armBottomRollerPivot.append(bottomRoller2);
        armBottomRollerPivot.append(bottomRoller3);
        armBottomRollerPivot.append(bottomRoller4);

        /* SUPERSTRUCTURE VISUALIZER */
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

        armTopRollerPivot.setPosition(width/2 + Math.cos(Units.degreesToRadians(arm.getAngle())) * Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_ROLLER, elevatorHeight + Math.sin(Units.degreesToRadians(arm.getAngle())) * Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_ROLLER);
        armBottomRollerPivot.setPosition(width/2 + Math.cos(Units.degreesToRadians(arm.getAngle())) * Constants.Arm.DISTANCE_FROM_PIVOT_TO_BOTTOM_ROLLER, elevatorHeight + Math.sin(Units.degreesToRadians(arm.getAngle())) * Constants.Arm.DISTANCE_FROM_PIVOT_TO_BOTTOM_ROLLER);

        SmartDashboard.putData("Visualizers/Robot", canvas);
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

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public void updateElevatorHeight(double elevatorHeight) {
        this.elevatorHeight = elevatorHeight;
        elevatorCarriageTopFront.setPosition((width + Constants.Elevator.WIDTH) / 2 - Units.inchesToMeters(2), elevatorHeight);
        elevatorCarriageTopBack.setPosition((width - Constants.Elevator.WIDTH) / 2 + Units.inchesToMeters(2), elevatorHeight);

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

}
