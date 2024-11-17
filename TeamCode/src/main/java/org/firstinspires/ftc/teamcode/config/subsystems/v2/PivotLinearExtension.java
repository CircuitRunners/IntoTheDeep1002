package org.firstinspires.ftc.teamcode.config.subsystems.v2;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotLinearExtension {
    // Hardware
    private final DcMotorEx pivotMotor;
    private final DcMotorEx extensionMotor;
    private final AnalogInput pivotEncoder;

    // PIDF Controllers
    private final PIDFController pivotPID;
    private final PIDFController extensionPID;

    // PID Coefficients
    public static double PIVOT_KP = 0.02, PIVOT_KI = 0.0, PIVOT_KD = 0.001, PIVOT_KF = 0.0;
    public static double EXTENSION_KP = 0.01, EXTENSION_KI = 0.0, EXTENSION_KD = 0.001, EXTENSION_KF = 0.0;

    // Gravity Compensation
    private static final double GRAVITY_CONSTANT = 0.1;
    private static final double MAX_EXTENSION = 100.0;

    // State Variables
    private double pivotTargetAngle = 0.0;
    private double extensionTargetLength = 0.0;

    public PivotLinearExtension(HardwareMap hardwareMap) {
        pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extensionMotor = hardwareMap.get(DcMotorEx.class, "extension");
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivot_enc");

        pivotPID = new PIDFController(PIVOT_KP, PIVOT_KI, PIVOT_KD, PIVOT_KF);
        extensionPID = new PIDFController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD, EXTENSION_KF);

        pivotTargetAngle = getPivotAngle();
        extensionTargetLength = getExtensionLength();
    }

    public void update() {
        // Gravity compensation for pivot
        double extensionScale = getExtensionLength() / MAX_EXTENSION;
        double pivotGravityCompensation = GRAVITY_CONSTANT * Math.cos(Math.toRadians(getPivotAngle())) * (1+extensionScale);

        // Gravity compensation for extension
        double extensionGravityCompensation = GRAVITY_CONSTANT * Math.sin(Math.toRadians(getPivotAngle()));

        // Calculate motor outputs
        double pivotOutput = pivotPID.calculate(getPivotAngle(), pivotTargetAngle) + pivotGravityCompensation;
        double extensionOutput = extensionPID.calculate(getExtensionLength(), extensionTargetLength) + extensionGravityCompensation;

        // Set motor outputs
        pivotMotor.setPower(pivotOutput);
        extensionMotor.setPower(extensionOutput);
    }

    public void setPivotTargetAngle(double angle) {
        pivotTargetAngle = angle;
    }

    public void setExtensionTargetLength(double length) {
        extensionTargetLength = length;
    }

    public double getPivotAngle() {
        return pivotEncoder.getVoltage() / 3.2 * 360;
    }

    public double getExtensionLength() {
        return extensionMotor.getCurrentPosition();
    }
}
