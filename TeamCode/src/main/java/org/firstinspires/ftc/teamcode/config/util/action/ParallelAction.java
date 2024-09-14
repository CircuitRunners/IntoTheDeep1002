package org.firstinspires.ftc.teamcode.config.util.action;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ParallelAction implements Action {
    private List<Action> actions;

    public ParallelAction(List<Action> actions) {
        this.actions = new ArrayList<>(actions);
    }

    public ParallelAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run(TelemetryPacket p) {
        actions.removeIf(action -> !action.run(p));
        return !actions.isEmpty();
    }

    @Override
    public void preview(Canvas fieldOverlay) {
        for (Action a : actions) {
            a.preview(fieldOverlay);
        }
    }
}