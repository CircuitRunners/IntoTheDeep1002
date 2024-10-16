package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    // Robot Characteristics
    public static final double WEIGHT = 7.983226;
    // Robot height
    public static final double WIDTH = 14.3582677;
    // Robot width
    public static final double LENGTH = 14.8937008;

    // OTOS Values
    public static final double Y_OFFSET = -2.83465;
    public static final double X_OFFSET = 0;
    public static final double PEDRO_Y_OFFSET = -2.83465;
    public static final double PEDRO_X_OFFSET = 0;
    public static final double PEDRO_H_OFFSET = 270; // In degrees
    public static final double L_SCALER = 1.0;
    public static final double A_SCALER = 0.98872139;


    // Arm Constants
    public static double armStart = 0.0;
    public static double ARM_TARGET = 0.0;
    public static double ARM_OFF = -2.01;
    public static double ARM_SPEED = 0.05;
    public static double ARM_MIN = -4.85;
    public static double ARM_MAX = 1.95;
    public static double ARM_LOWBASKET = -0.78;
    public static double ARM_INTAKE = -4.6;
    public static double ARM_CLEAR = -4.3;
    public static double ARM_SPECIMEN = -3;
    public static double ARM_SPECIMEN_SCORE = -2.95;
    public static double ARM_OBSERVATION = 1;
    public static double kP = 3;
    public static double kI = 0.02;
    public static double kD = 0.02;
    public static double f = -0.1;
    public static double TICK_PER_RAD = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * (1+(46.0/17.0)) * 28) / 2*Math.PI / 3.2;
}
