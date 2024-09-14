package org.firstinspires.ftc.teamcode.config.util.action;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class RunAction implements Action {

    private final Runnable runnable;
    private Runnable callback;

    public RunAction(Runnable runnable) {
        this.runnable = runnable;
    }

    public void runAction() {
        runnable.run();
        if (callback != null) {
            callback.run();
        }
    }

    public void setCallback(Runnable callback) {
        this.callback = callback;
    }

    // Adapter to make Action compatible with the Action interface
    public boolean run(TelemetryPacket p) {
        runAction();
        return false; // Regular actions complete after one execution
    }
}