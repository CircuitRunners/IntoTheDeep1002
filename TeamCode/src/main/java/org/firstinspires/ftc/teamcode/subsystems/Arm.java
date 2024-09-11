package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HWValues;

public class Arm {

    final double ARM_TICKS_PER_DEGREE = Constants.ARM_TICKS_PER_DEGREE;
    public final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    public final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    public final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    public final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    public final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    private final PIDController controller;

    public static double target = 0;

    private final DcMotorEx arm_motor;

    public Arm(HardwareMap hardwareMap) {
        controller = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
        arm_motor = hardwareMap.get(DcMotorEx.class, HWValues.ARM);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        controller.setPID(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ticks_in_degrees = Constants.ARM_TICKS_PER_DEGREE;
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * Constants.ARM_F;

        double power = pid + ff;

        arm_motor.setPower(power);
    }

    public void setArmPos(double position) {
        target = position;
    }

    public double getArmPos() {
        return arm_motor.getCurrentPosition();
    }

    public double getArmTarget() {
        return target;
    }
}
