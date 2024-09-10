package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HWValues;

public class Arm {
    private PIDController controller;

    public static int target = 0;

    private final double ticks_in_degrees = Constants.ARM_TICKS_PER_DEGREE;

    private DcMotorEx arm_motor;

    public Arm(HardwareMap hardwareMap) {
        controller = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);

        arm_motor = hardwareMap.get(DcMotorEx.class, HWValues.ARM);

    }

    public void armUpdate() {
        controller.setPID(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * Constants.ARM_F;

        double power = pid + ff;

        arm_motor.setPower(power);
    }
}
