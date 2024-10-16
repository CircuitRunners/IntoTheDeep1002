package org.firstinspires.ftc.teamcode.config.subsystems.v2;
import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.f;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.FeedForwardConstant;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.config.util.action.RunAction;

public class Pivot {
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double f = 0.0;
    private static final double TICKS_PER_DEGREE =

    public static CustomPIDFCoefficients PID = new CustomPIDFCoefficients(kP, kI, kD, new Pivot.PivotPIDF());

    static class PivotPIDF implements FeedForwardConstant {
        @Override
        public double getConstant(double input) {
            return Math.sin(input) * f;
        }
    }

    public DcMotorEx pivotMotor;

    // Runactions
    public RunAction pivotIntake, pivotOuttake, pivotUpright;

    public double
}
