package org.firstinspires.ftc.teamcode.threading.control;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.threading.UltroThread;

public class UltroImu extends UltroThread {
    private BNO055IMU imu;


    private Orientation lastAngles;
    private volatile double globalAngle;
    private double roll = 0, pitch = 0, yaw = 0;

    public UltroImu() {
        super();
    }

    public void setUp(DeviceMap map) {
        if(map.getImu() == null) return;
        this.imu = map.getImu();
        lastAngles = getOrientation();
    }

    private void updateAngles(){
        Quaternion quatAngles = imu.getQuaternionOrientation();

        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;

        roll = Math.atan2( 2*(w*x + y*z) , 1 - (2*(x*x + y*y)) ) * 180.0 / Math.PI;
        pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        yaw = Math.atan2( 2*(w*z + x*y), 1 - (2*(y*y + z*z)) ) * 180.0 / Math.PI;
    }
    private void updateAngles2() {
        Orientation angles = getOrientation();


        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        incrementGlobalAngle(deltaAngle);

        lastAngles = angles;
    }

    private synchronized void incrementGlobalAngle(double deltaAngle) {
        globalAngle = globalAngle + deltaAngle;
    }

    @Override
    public void go() {
        updateAngles2();
    }

    public double getAngle() {
        return globalAngle;
    }

    public void resetAngle() {
        lastAngles = getOrientation();
        globalAngle = 0;
    }


    private Orientation getOrientation() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double getRoll() {
        return roll;
    }

    public double getPitch() {
        return pitch;
    }

    public double getYaw() {
        return yaw;
    }
}
