package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState;

public class ElevatorClawCommand extends CommandBase {
    public enum ClawPositions {
        CLOSED,
        OPEN
    }

    Elevator elevator;
    ClawPositions clawPosition;

    public ElevatorClawCommand(Elevator elevator, ClawPositions clawPosition) {
        this.elevator = elevator;
        this.clawPosition = clawPosition;
    }

    @Override
    public void initialize() {
        switch (clawPosition) {
            case OPEN:
                elevator.openClaw();
                break;
            case CLOSED:
                elevator.closeClaw();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        switch (clawPosition) {
            case CLOSED:
                return !elevator.handIsOpen;
            case OPEN:
                return elevator.handIsOpen;
        }
        return false;
    }
}
