package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
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

    private double elevatorHeight;

    private final MechanismRoot2d elevatorFixedStageBottomFront;
    private final MechanismRoot2d elevatorFixedStageBottomBack;
    private final MechanismRoot2d elevatorCarriageTopFront;
    private final MechanismRoot2d elevatorCarriageTopBack;

    private final MechanismLigament2d[] elevatorParts;

    private final MechanismRoot2d armPivot;
    private final MechanismLigament2d arm;

    private final MechanismRoot2d topRollerPivot;
    private final MechanismRoot2d bottomRollerPivot;

    private final MechanismLigament2d[] topRollers;
    private final MechanismLigament2d[] bottomRollers;

    private final MechanismRoot2d coralRoot;
    private final MechanismLigament2d coralFront;
    private final MechanismLigament2d coralBack;

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
        elevatorHeight = Constants.Elevator.MIN_HEIGHT_METERS;

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

        MechanismLigament2d frontFixedStage = new MechanismLigament2d(
            "Front Fixed Stage",
            Constants.Elevator.FIXED_STAGE_MAX_HEIGHT,
            90,
            10,
            new Color8Bit(Color.kOrange)
        );
        elevatorFixedStageBottomFront.append(frontFixedStage);

        MechanismLigament2d backFixedStage = new MechanismLigament2d(
            "Back Fixed Stage",
            Constants.Elevator.FIXED_STAGE_MAX_HEIGHT,
            90,
            10,
            new Color8Bit(Color.kOrange)
        );
        elevatorFixedStageBottomBack.append(backFixedStage);

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

        MechanismLigament2d frontCarriage = new MechanismLigament2d(
            "Front Carriage",
            Constants.Elevator.CARRIAGE_LENGTH,
            -90,
            10,
            new Color8Bit(Color.kOrange)
        );
        elevatorCarriageTopFront.append(frontCarriage);

        MechanismLigament2d backCarriage = new MechanismLigament2d(
            "Back Carriage",
            Constants.Elevator.CARRIAGE_LENGTH,
            -90,
            10,
            new Color8Bit(Color.kOrange)
        );
        elevatorCarriageTopBack.append(backCarriage);

        MechanismLigament2d topCarriage = new MechanismLigament2d(
            "Top Carriage",
            Constants.Elevator.WIDTH - (2 * Units.inchesToMeters(2)),
            -180,
            10,
            new Color8Bit(Color.kOrange)
        );
        elevatorCarriageTopFront.append(topCarriage);

        elevatorParts = new MechanismLigament2d[]{frontFixedStage, backFixedStage, frontCarriage, backCarriage, topCarriage};

        /* CORAL VISUALIZER */
        coralRoot = canvas.getRoot(
            "Coral", 
            width / 2, 
            elevatorHeight - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR - (Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_TOP_ROLLER + Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_BOTTOM_ROLLER) / 2);
        
        coralFront = new MechanismLigament2d("Coral Front", 0.1, 90, 20, new Color8Bit(Color.kWhite));
        coralRoot.append(coralFront);
        coralBack = new MechanismLigament2d("Coral Back", 0.1, -90, 20, new Color8Bit(Color.kWhite));
        coralRoot.append(coralBack);
    }

    public void updateArmAngle(Rotation2d armAngle, boolean atTargetAngle) {
        armPivot.setPosition(width/2, elevatorHeight - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR);
        arm.setAngle(armAngle);

        arm.setColor(atTargetAngle ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kRed));

        topRollerPivot.setPosition(
            width/2 + Math.cos(Units.degreesToRadians(arm.getAngle())) * Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_TOP_ROLLER, 
            elevatorHeight - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR + Math.sin(Units.degreesToRadians(arm.getAngle())) * Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_TOP_ROLLER);
        bottomRollerPivot.setPosition(
            width/2 + Math.cos(Units.degreesToRadians(arm.getAngle())) * Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_BOTTOM_ROLLER, 
            elevatorHeight - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR + Math.sin(Units.degreesToRadians(arm.getAngle())) * Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_BOTTOM_ROLLER);

        coralRoot.setPosition(
            width/2 + Math.cos(Units.degreesToRadians(arm.getAngle())) * ((Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_TOP_ROLLER + Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_BOTTOM_ROLLER) / 2), 
            elevatorHeight - Constants.Arm.DISTANCE_FROM_PIVOT_TO_TOP_OF_ELEVATOR + Math.sin(Units.degreesToRadians(arm.getAngle())) * ((Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_TOP_ROLLER + Constants.Shooter.DISTANCE_FROM_ARM_PIVOT_TO_BOTTOM_ROLLER) / 2));

        coralFront.setAngle(90 + armAngle.getDegrees());
        coralBack.setAngle(-90 + armAngle.getDegrees());

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public void updateShooter(double speed, boolean hasCoral) {
        for (int i = 0; i < 4; i++) {
            topRollers[i].setAngle(topRollers[i].getAngle() + speed * 10);
            bottomRollers[i].setAngle(bottomRollers[i].getAngle() - speed * 10);
        }

        coralFront.setLength(hasCoral ? 0.1 : 0);
        coralBack.setLength(hasCoral ? 0.1 : 0);

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

    public void updateElevatorHeight(double elevatorHeight, boolean atTargetHeight) {
        this.elevatorHeight = elevatorHeight;
        elevatorCarriageTopFront.setPosition((width + Constants.Elevator.WIDTH) / 2 - Units.inchesToMeters(2), elevatorHeight);
        elevatorCarriageTopBack.setPosition((width - Constants.Elevator.WIDTH) / 2 + Units.inchesToMeters(2), elevatorHeight);

        for (MechanismLigament2d elevatorPart : elevatorParts) {
            elevatorPart.setColor(atTargetHeight ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kRed));
        }

        SmartDashboard.putData("Visualizers/Robot", canvas);
    }

}
