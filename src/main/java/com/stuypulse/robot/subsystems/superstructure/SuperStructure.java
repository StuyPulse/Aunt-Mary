package com.stuypulse.robot.subsystems.superstructure;

import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.arm.Arm.ArmState;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase{
    private static final SuperStructure instance;
    
    static { 
        instance = new SuperStructure();
    }

    public static SuperStructure getInstance() {
        return instance;
    }

    public enum SuperStructureTargetState {
        FEED(ArmState.FEED, ElevatorState.FEED),
        STOW(ArmState.STOW, ElevatorState.STOW),
        L2_FRONT(ArmState.L2_FRONT, ElevatorState.L2_FRONT),
        L2_BACK(ArmState.L2_BACK, ElevatorState.L2_BACK),
        L3_FRONT(ArmState.L3_FRONT, ElevatorState.L3_FRONT),
        L3_BACK(ArmState.L3_BACK, ElevatorState.L3_BACK),
        L4_FRONT(ArmState.L4_FRONT, ElevatorState.L4_FRONT),
        L4_BACK(ArmState.L4_BACK, ElevatorState.L4_BACK),
        ALGAE_L2(ArmState.ALGAE_L2, ElevatorState.ALGAE_L2),
        ALGAE_L3(ArmState.ALGAE_L3, ElevatorState.ALGAE_L3),
        BARGE(ArmState.BARGE, ElevatorState.BARGE);

        private ArmState targetArmState;
        private ElevatorState targetElevatorState;

        private SuperStructureTargetState(ArmState targetArmState, ElevatorState targetElevatorState) {
            this.targetArmState = targetArmState;
            this.targetElevatorState = targetElevatorState;
        }

        public ArmState getTargetArmState() {
            return this.targetArmState;
        }

        public ElevatorState getTargetElevatorState() {
            return this.targetElevatorState;
        }
    }

    private SuperStructureTargetState targetState;

    protected SuperStructure() {
        this.targetState = SuperStructureTargetState.STOW;
    }

    public void setTargetState(SuperStructureTargetState state) {
        this.targetState = state;
    }

    public SuperStructureTargetState getTargetState() {
        return this.targetState;
    }

    @Override
    public void periodic() {
        Arm arm = Arm.getInstance();
        Elevator elevator = Elevator.getInstance();

        ArmState currentArmState = arm.getState();
        ElevatorState currentElevatorState = elevator.getState();

        Rotation2d currentArmAngle = arm.getCurrentAngle();
        double currentElevatorHeight = elevator.getCurrentHeight();
    }
}
