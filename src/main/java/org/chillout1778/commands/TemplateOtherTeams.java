package org.chillout1778.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TemplateOtherTeams extends Command {
    public void startDriving() {
        // TODO: adapt for other teams
    }

    public void stopDriving() {
        // TODO: adapt for other teams
    }

    private final Timer timer = new Timer();

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2.0);
    }

    @Override
    public void execute() {
        // The reason this is inside execute() (meaning it runs every single tick)
        // is because normally you need to call Swerve.drive() functions continually
        // otherwise they won't update properly.
        startDriving();
    }

    @Override
    public void end(boolean wasCanceled) {
        stopDriving();
    }
}
