package org.firstinspires.ftc.teamcode.config.subsystems;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.util.HWValues;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EndEffector {
    private final Servo claw;
    private final Servo diffy1;
    private final Servo diffy2;

    public EndEffector(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, HWValues.CLAW);
        diffy1 = hardwareMap.get(Servo.class, HWValues.DIFFY1);
        diffy2 = hardwareMap.get(Servo.class, HWValues.DIFFY2);
        closeClaw();
        idlePosition();
    }

    public void telemetry (Telemetry telemetry) {
        telemetry.addData("diffy1 pos", diffy1.getPosition());
        telemetry.addData("diffy2 pos", diffy2.getPosition());
        telemetry.addData("claw pos", claw.getPosition());
    }

    public void openClaw() {
        claw.setPosition(0);
    }

    public void closeClaw() {
        claw.setPosition(1);
    }

    public void pickupPosition() {
        diffy1.setPosition(0.89);
        diffy2.setPosition(0.86);
    }

    public void idlePosition() {
        diffy1.setPosition(0.89);
        diffy2.setPosition(0.86);
    }

    public void diffyLeft() {
        diffy1.setPosition(0.72);
        diffy2.setPosition(0.85);
    }

    public void diffyRight() {
        diffy1.setPosition(0.78);
        diffy2.setPosition(0.79);
    }

    public void transferPosition() {
        diffy1.setPosition(0.92);
        diffy2.setPosition(0.89);
    }

    public void placingPosition() {
        diffy1.setPosition(0.75);
        diffy2.setPosition(0.82);
    }

    public void depositLoop(Gamepad gamepad) {
        if (gamepad.y) {
            openClaw();
        } else if (gamepad.b) {
            closeClaw();
        } else if (gamepad.dpad_left) {
            pickupPosition();
        } else if (gamepad.dpad_right) {
            idlePosition();
        } else if (gamepad.right_bumper) {
            transferPosition();
        } else if (gamepad.x) {
            placingPosition();
        }
    }

    public void diffyTurning(Gamepad gamepad) {
        if (gamepad.x) {
            diffyLeft();
        } else if (gamepad.b) {
            diffyRight();
        }
    }


}
