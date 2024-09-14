package org.firstinspires.ftc.teamcode.config.util.action;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public interface Action {
    boolean run(TelemetryPacket p);

    default void preview(Canvas fieldOverlay) {}
}

