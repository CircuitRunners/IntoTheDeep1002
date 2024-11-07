package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.AnalogInput;



@TeleOp(name = "EncoderTest", group = "Test")
public class EncoderTest extends OpMode {
    AnalogInput encoder = hardwareMap.get(AnalogInput.class, "enc");
    @Override
    public void init() {
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        double position = encoder.getVoltage() / 3.2 * 360;

        telemetry.addData("Position", position);
        telemetry.update();
    }


}