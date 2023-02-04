package org.firstinspires.ftc.teamcode.subsystems.slide;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.slide.ArmPositionCommand;
import org.firstinspires.ftc.teamcode.commands.slide.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.slide.LiftPositionCommand;

public class SlideManager {

    private final SlideSubsystem subsystem;

    public STATE state = STATE.TRANSITION;
    public STATE previousState = STATE.IDLE;

    public enum STATE {
        IDLE,
        INTAKE,
        TRANSITION,
        LOW_BACK,
        MEDIUM,
        HIGH
    }

    public SlideManager(SlideSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    public void reset() {
        intake();
    }

    public void intake() {
        setState(STATE.INTAKE);
        schedule(new ParallelCommandGroup(
                new InstantCommand(() -> {
                    subsystem.update(SlideSubsystem.PivotState.FLAT);
                }),
                new ArmPositionCommand(subsystem,
                        SlideSubsystem.ArmState.INTAKE, 1000),
                new LiftPositionCommand(subsystem, 0, 20, 1500, SlideSubsystem.STATE.FAILED_RETRACT),
                new ClawCommand(subsystem, SlideSubsystem.ClawState.OPEN)
        ));
    }

    public void transition() {
        transition(STATE.TRANSITION);
    }

    public void transition(STATE postState) {
        setState(STATE.TRANSITION);
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    subsystem.update(SlideSubsystem.PivotState.FLAT);
                }),
                new ClawCommand(subsystem, SlideSubsystem.ClawState.CLOSED),
                new WaitCommand(200),
                new ArmPositionCommand(subsystem, SlideSubsystem.ArmState.TRANSITION, 500),
                new InstantCommand(() -> {
                    setup(postState);
                })
        ));
    }

    public void setup(STATE state) {
        switch (state) {
            case LOW_BACK: {
                schedule(new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            subsystem.update(SlideSubsystem.PivotState.FLAT);
                        }),
                        new LiftPositionCommand(subsystem, 0, 20, 1500, SlideSubsystem.STATE.FAILED),
                        new ClawCommand(subsystem, SlideSubsystem.ClawState.CLOSED),
                        new ArmPositionCommand(subsystem, SlideSubsystem.ArmState.SCORE_LOW, 1000)
                ));
                break;
            }
            case MEDIUM: {
                schedule(new ParallelCommandGroup(
                        new LiftPositionCommand(subsystem, 1600, 20, 1500, SlideSubsystem.STATE.FAILED_EXTEND),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new InstantCommand(() -> {
                                    subsystem.update(SlideSubsystem.PivotState.AIM);
                                }),
                                new ArmPositionCommand(subsystem, SlideSubsystem.ArmState.AIM, 1000)
                        )
                ));
                break;
            }
            case HIGH: {
                schedule(new ParallelCommandGroup(
                        new LiftPositionCommand(subsystem, 2400, 20, 2000, SlideSubsystem.STATE.FAILED_EXTEND),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new InstantCommand(() -> {
                                    subsystem.update(SlideSubsystem.PivotState.AIM);
                                }),
                                new ArmPositionCommand(subsystem, SlideSubsystem.ArmState.AIM, 1000)
                        )
                ));
                break;
            }
        }
        setState(state);
    }

    public void score() {
        switch (this.state) {
            case LOW_BACK:
            case TRANSITION: {
                schedule(new SequentialCommandGroup(
                        new ClawCommand(subsystem, SlideSubsystem.ClawState.OPEN),
                        new WaitCommand(100)
                                .andThen(new InstantCommand(this::transition))));
                break;
            }
            case MEDIUM:
            case HIGH: {
                schedule(new SequentialCommandGroup(
                        new ArmPositionCommand(subsystem, SlideSubsystem.ArmState.SCORE, 500),
                        new ClawCommand(subsystem, SlideSubsystem.ClawState.OPEN),
                        new WaitCommand(100),
                        new ClawCommand(subsystem, SlideSubsystem.ClawState.CLOSED),
                        new WaitCommand(150),
                        new ArmPositionCommand(subsystem, SlideSubsystem.ArmState.TRANSITION, 700),
                        new InstantCommand(this::intake)
                ));
                break;
            }
        }
    }

    public void setState(STATE state) {
        this.previousState = this.state;
        this.state = state;
    }


    public void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
    }
}
