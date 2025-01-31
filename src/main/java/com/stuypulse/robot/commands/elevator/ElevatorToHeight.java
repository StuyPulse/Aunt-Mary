package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.stuylib.streams.booleans.BStream;

import edu.wpi.first.wpilibj2.command.InstantCommand;   

public class ElevatorToHeight extends InstantCommand {
    private final Elevator elevator;
    private final double targetHeight;
    private final BStream belowTop, aboveBottom;

    public ElevatorToHeight(double targetHeight){
        elevator = Elevator.getInstance();
        this.targetHeight = targetHeight;
        
        belowTop = BStream.create(() -> !elevator.atTop())
                .and(() -> targetHeight < Constants.Elevator.MAX_HEIGHT_METERS);
        aboveBottom = BStream.create(() -> !elevator.atBottom())
                .and(() -> targetHeight > Constants.Elevator.MIN_HEIGHT_METERS);

        addRequirements(elevator);
    }

    public void initialize(){
        if (belowTop.get() && aboveBottom.get()) {
            elevator.setTargetHeight(targetHeight);
        }
    }
}
