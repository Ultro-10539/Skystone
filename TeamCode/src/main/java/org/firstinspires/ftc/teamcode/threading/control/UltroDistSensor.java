package org.firstinspires.ftc.teamcode.threading.control;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.monitor.RobotData;
import org.firstinspires.ftc.teamcode.threading.UltroThread;

public class UltroDistSensor extends UltroThread {
    private DistanceSensor left, right, back, colorDLeft, colorDRight;

    public UltroDistSensor() {
        super();
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
        RobotData.updateDistanceValues(
            left.getDistance(DistanceUnit.CM),
            right.getDistance(DistanceUnit.CM),
            back.getDistance(DistanceUnit.CM),
            colorDLeft.getDistance(DistanceUnit.CM),
            colorDRight.getDistance(DistanceUnit.CM));
    }

}
