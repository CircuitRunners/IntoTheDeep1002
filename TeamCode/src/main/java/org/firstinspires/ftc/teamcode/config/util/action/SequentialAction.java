package org.firstinspires.ftc.teamcode.config.util.action;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SequentialAction implements Action {
    private List<Action> actions;

    public SequentialAction(List<Action> actions) {
        this.actions = new ArrayList<>(actions);
    }

    public SequentialAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run(TelemetryPacket p) {
        if (actions.isEmpty()) {
            return false;
        }

        if (actions.get(0).run(p)) {
            return true;
        } else {actions.remove(0);
            return run(p);
        }
    }

    @Override
    public void preview(Canvas fieldOverlay) {
        for (Action a : actions) {
            a.preview(fieldOverlay);
        }
    }


}