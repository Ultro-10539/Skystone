package org.firstinspires.ftc.teamcode.threading.control;

import com.qualcomm.robotcore.hardware.DcMotor;

import net.jafama.FastMath;

import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.threading.UltroThread;

import java.util.concurrent.atomic.AtomicBoolean;

public class DriveThread extends UltroThread {
    private final MotorData[] data;
    private final Runnable clear;

    public DriveThread() {
        super();

        DeviceMap map = DeviceMap.getInstance();
        MotorData leftTop = new MotorData(map.getLeftTop());
        MotorData rightTop = new MotorData(map.getRightTop());
        MotorData leftBottom = new MotorData(map.getLeftBottom());
        MotorData rightBottom = new MotorData(map.getRightBottom());

        data = new MotorData[]{leftTop, rightTop, leftBottom, rightBottom};
        clear = map::clearBulkCache;
    }

    @Override
    public void setUp(DeviceMap map) {
    }

    private class MotorData {
        private final DcMotor motor;
        private double power;
        private double pos;
        private double finalPos;
        //false = power does not need to be updated
        //true = power needs updating
        private final AtomicBoolean checkPower;

        private MotorData(DcMotor motor) {
            this.motor = motor;
            this.checkPower = new AtomicBoolean(true);
        }


    }

    @Override
    public void go() {
        synchronized (data) {
            clear.run();
            for(MotorData d : data) {
                //positions
                d.pos = d.motor.getCurrentPosition();

                if(FastMath.abs(d.finalPos) > FastMath.abs(d.pos)) {
                    d.checkPower.set(true);
                    d.power = 0;
                }
                //set power
                if(d.checkPower.get()) continue;
                d.checkPower.set(true);
                d.motor.setPower(d.power);
            }
        }
    }


    public void setPowers(double... powers) {
        synchronized (data) {
            for(int i = 0, size = data.length; i < size; i++) {
                MotorData d = data[i];
                double power = powers[i];
                if(d.power == power) continue;
                if(!d.checkPower.get()) continue;
                d.checkPower.set(false);
                d.power = power;
            }
        }
    }

    public double[] getCurrentPosition() {
        int size = data.length;
        double[] doubles = new double[size];
        for(int i = 0; i < size; i++) {
            doubles[i] = data[i].pos;
        }
        return doubles;
    }
    public void setPosition(double[] positions) {
        synchronized (data) {
            for(int i = 0, size = positions.length; i < size; i++) {
                data[i].finalPos = positions[i];
            }
        }
    }
}
