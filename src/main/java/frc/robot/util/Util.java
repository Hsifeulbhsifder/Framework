// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class Util {

    public static double lerp(double a, double b, double alpha) {
        return a * (1.0 - alpha) + b * alpha;
    }

    public static double unlerp(double a, double b, double value) {
        return (value - a) / (b - a);
    }

}
