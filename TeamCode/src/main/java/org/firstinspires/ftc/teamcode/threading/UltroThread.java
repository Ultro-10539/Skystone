package org.firstinspires.ftc.teamcode.threading;

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

    public abstract void setUp(DeviceMap map);

    public TimeUnit getTimeUnit() {
        return TimeUnit.MILLISECONDS;
    }
    public long getTime() {
        return 50L;
    }
}
