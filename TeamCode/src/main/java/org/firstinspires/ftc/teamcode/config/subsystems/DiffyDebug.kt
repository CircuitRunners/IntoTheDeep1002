package org.firstinspires.ftc.teamcode.lib

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

class DiffyDebug (hardwareMap: HardwareMap) {
    val claw: Servo
    val diffy1: Servo
    val diffy2: Servo
    val arm1: Servo
    val arm2: Servo


    init {
        claw = hardwareMap.get(Servo::class.java, "claw")
        diffy1 = hardwareMap.get(Servo::class.java, "diffy1")
        diffy2 = hardwareMap.get(Servo::class.java, "diffy2")
        diffy2.direction = Servo.Direction.REVERSE
        arm1 = hardwareMap.get(Servo::class.java, "arm1")
        arm2 = hardwareMap.get(Servo::class.java, "arm2")
        closeClaw()
        idlePosition()
    }

    fun telemetry(telemetry: Telemetry){
        telemetry.addData("diffy1 pos", diffy1.position)
        telemetry.addData("diffy2 pos", diffy2.position)
        telemetry.addData("arm1 pos", arm1.position)
        telemetry.addData("arm2 pos", arm2.position)
        telemetry.addData("claw pos", claw.position)
    }
    fun openClaw() {
        claw.position = 0.0
    }

    fun closeClaw() {
        claw.position = 1.0
    }

    fun pickupPosition() {
        diffy1.position = 0.89
        diffy2.position = 0.86
        arm1.position = 0.76
        arm2.position = 0.76
    }

    fun idlePosition() {
        diffy1.position = 0.89
        diffy2.position = 0.86
        arm1.position = 0.6
        arm2.position = 0.6
    }

    fun transferPosition() {
        arm1.position = 0.7
        arm2.position = 0.7
        diffy1.position = 0.92
        diffy2.position = 0.89
    }

    fun placingPosition() {
        arm1.position = 0.0
        arm2.position = 0.0
        diffy1.position = 0.75
        diffy2.position = 0.82
    }

    fun depositLoop(gamepad: Gamepad) {
        if (gamepad.y)
            openClaw()
        else if (gamepad.b)
            closeClaw()
        else if (gamepad.dpad_left)
            pickupPosition()
        else if (gamepad.dpad_right)
            idlePosition()
        else if (gamepad.right_bumper)
            transferPosition()
        else if (gamepad.x)
            placingPosition()
    }

    fun diffyAdjustmentRight() {
        diffy1.position += 0.0001
        diffy2.position -= 0.0001
    }

    fun diffyAdjustmentLeft() {
        diffy1.position -= 0.0001
        diffy2.position += 0.0001
    }
    fun diffyTurning(gamepad: Gamepad) {
        if (gamepad.x)
            diffyAdjustmentLeft()
        else if (gamepad.b)
            diffyAdjustmentRight()
    }
}