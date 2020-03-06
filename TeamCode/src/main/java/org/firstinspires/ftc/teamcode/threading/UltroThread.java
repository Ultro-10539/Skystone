package org.firstinspires.ftc.teamcode.threading;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.monitor.DeviceMap;

import java.util.concurrent.TimeUnit;

/**
 * PRE REQUISITES: DEFAULT CONSTRUCTOR ONLY
 */
public abstract class UltroThread implements Runnable {

    public UltroThread() {
        DeviceMap deviceMap = DeviceMap.getInstance();
        setUp(deviceMap);
    }

    @Override
    public void run() {
        while(!Thread.currentThread().isInterrupted()) {
            try {
                go();
            } catch (Exception e) {
                e.printStackTrace();
                Thread.currentThread().interrupt();
            }
        }
    }

    public abstract void go();

    public abstract void setUp(DeviceMap map);

    public TimeUnit getTimeUnit() {
        return TimeUnit.MILLISECONDS;
    }

    public long getTime() {
        return 5L;
    }

    void interrupt() {
        Thread.currentThread().interrupt();
    }
}
