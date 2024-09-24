package org.firstinspires.ftc.teamcode.config.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.util.HWValues;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import org.firstinspires.ftc.teamcode.config.util.action.RunAction;


public class EndEffector {
    private Telemetry telemetry;
    private final Servo claw;
    private final Servo diffy1;
    private final Servo diffy2;

    // TODO
    public RunAction actions;

    public EndEffector(HardwareMap hardwareMap, Telemetry telemetry) {
        claw = hardwareMap.get(Servo.class, HWValues.CLAW);
        diffy1 = hardwareMap.get(Servo.class, HWValues.DIFFY1);
        diffy2 = hardwareMap.get(Servo.class, HWValues.DIFFY2);
        closeClaw();
        idlePosition();
    }

    public double getClawPosition() {
        return claw.getPosition();
    }

    public double getDiffy1Position() {
        return diffy1.getPosition();
    }
    public double getDiffy2Position() {
        return diffy2.getPosition();
    }


    public void openClaw() {
        claw.setPosition(0.4);
    }

    public void closeClaw() {
        claw.setPosition(0);
    }
    public void idlePosition() {
        diffy1.setPosition(0.43);
        diffy2.setPosition(0.38);
    }

    public void specimenPositionH() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }
    public void specimenPositionV() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }
    public void specimenPositionAL() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }
    public void specimenPositionAR() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }

    public void subPosition() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }
    public void basketPosition() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }
    public void obsPosition() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }

    public void hangPosition() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }


}
