package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.Locale;

public class AngleConverter {
    private double[] movement;
    private AngleConverter(double magnitude, double angle) {
        RobotLog.d(String.format(Locale.ENGLISH, "Magnitude: %f, Angle: %f", magnitude, angle));
        this.setUp(magnitude, Math.toRadians(angle));
    }

    private void setUp(double magnitude, double angle) {
        double sin = magnitude * Math.sin(angle);
        double cos = magnitude * Math.cos(angle);
        this.movement = new double[]{cos,sin,sin,cos};
        RobotLog.d(String.format(Locale.ENGLISH, "Movement: %f %f %f %f",
                movement[0], movement[1], movement[2], movement[3]));

        //
    }
    public double getLeftTop() {
        return movement[0];
    }
    public double getRightTop() {
        return movement[1];
    }
    public double getLeftBottom() {
        return movement[2];
    }
    public double getRightBottom() {
        return movement[3];
    }

    public double[] getMovement() {
        return movement;
    }

    public static AngleConverter fromAngle(double x, double y) {
        if(Double.compare(x, 0) == 0 && Double.compare(y, 0) == 0) return null;
        return new AngleConverter(Math.sqrt(x * x + y * y), Math.atan2(y, x));
    }
}
