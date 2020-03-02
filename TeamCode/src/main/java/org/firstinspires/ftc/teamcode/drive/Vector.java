package org.firstinspires.ftc.teamcode.drive;

import android.os.Build;

import net.jafama.FastMath;

/**
 * They return completely new classes for immutability.
 */
public final class Vector {
    private double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector from(double x, double y) {
        return new Vector(x, y);
    }
    public double getX() {
        return x;
    }
    public Vector setX(double x) {
        return new Vector(x, y);
    }

    public double getY() {
        return y;
    }
    public Vector setY(double y) {
        return new Vector(x, y);
    }

    public Vector add(double x, double y) {
        return new Vector(this.x + x, this.y + y);
    }
    public Vector multiply(double scalar) {
        return new Vector(this.x * scalar, this.y * scalar);
    }
    public Vector divide(double scalar) {
        return new Vector(this.x / scalar, this.y / scalar);
    }

    /**
     * or magnitude
     * @return
     */
    public double lengthSquared() {
        return FastMath.pow2(this.x) + FastMath.pow2(this.y);
    }
    public double length() {
        return FastMath.hypot(this.x, this.y);
    }
    public Vector normalize() {
        double length = length();
        return new Vector(this.x / length, this.y / length);
    }

    @Override
    public String toString() {
        return "Vector{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }
}
