package org.firstinspires.ftc.teamcode.threading.control;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.monitor.RobotData;
import org.firstinspires.ftc.teamcode.threading.UltroThread;

public class UltroDistSensor extends UltroThread {
    private DistanceSensor left, right, back, colorDLeft, colorDRight;
    private final DistanceData data;
    private class DistanceData {
        private double left, right, back, colorLeft, colorRight;
    }

    public UltroDistSensor() {
        super();
        this.data = new DistanceData();
    }

    @Override
    public void setUp(DeviceMap map) {
        left = map.getDistanceLeft();
        right = map.getDistanceRight();
        back = map.getDistanceBack();
        colorDLeft = map.getSensorColorLeftDist();
        colorDRight = map.getSensorColorRightDist();
    }

    @Override
    public void go() {
        synchronized (data) {
            data.left = left.getDistance(DistanceUnit.CM);
            data.right = right.getDistance(DistanceUnit.CM);
            data.back = back.getDistance(DistanceUnit.CM);
            data.colorLeft = colorDLeft.getDistance(DistanceUnit.CM);
            data.colorRight = colorDRight.getDistance(DistanceUnit.CM);
        }
    }

    public DistanceData getData() {
        return data;
    }
}
