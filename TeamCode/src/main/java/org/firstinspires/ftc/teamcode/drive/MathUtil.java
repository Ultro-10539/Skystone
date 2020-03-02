package org.firstinspires.ftc.teamcode.drive;

import net.jafama.FastMath;

public final class MathUtil {
    public static double convert180to360(double angle) {
        if(angle > 0) return angle;
        else return 360 + angle;
    }
    public static float convert180to360(float angle) {
        if(angle > 0) return angle;
        else return 360 + angle;
    }

    public static float limit360(float angle) {
        if(angle < 360) return angle;
        if(angle > 360) {
            angle -= 360;
        }

        return limit360(angle);
    }
    public static double limit360(double angle) {
        if(angle < 360) return angle;
        if(angle > 360) {
            angle -= 360;
        }

        return limit360(angle);
    }

    public static double wrapAngle(double angle) {
        double pi180 = 180;
        if (-pi180 <= angle && angle <= pi180) {
            return angle;
        }


        //181 => =-179
        //-181 => 179

        // 1 = 181 mod 180
        double a = angle % pi180;
        return sign(-a) * 180 + a;
    }

    public static double sign(double number) {
        if(number == 0) return 0;
        else if(number > 0) return 1;
        else return -1;
    }
 }
