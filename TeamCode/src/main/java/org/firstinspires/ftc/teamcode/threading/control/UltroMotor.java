package org.firstinspires.ftc.teamcode.threading.control;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.monitor.RobotData;
import org.firstinspires.ftc.teamcode.threading.UltroThread;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

public class UltroMotor extends UltroThread {
    private DcMotor leftTop, leftBottom, rightTop, rightBottom,
            leftIntake, rightIntake, conveyer, lift;

    private ExpansionHubEx hubEx3, hubEx2;
    public UltroMotor() {
        super();
    }

    @Override
    public void setUp(DeviceMap map) {
        leftTop = map.getLeftTop();
        leftBottom = map.getLeftBottom();
        rightTop = map.getRightTop();
        rightBottom = map.getRightBottom();
        leftIntake = map.getLeftIntake();
        rightIntake = map.getRightIntake();
        conveyer = map.getConveyer();
        lift = map.getLift();

        hubEx3 = map.getExpansionHub3();
        hubEx2 = map.getExpansionHub2();

    }

    @Override
    public void go() {
        slowUpdate();
    }

    private void experimentalUpdate() {
        if(hubEx3 == null) {
            slowUpdate();
            return;
        }
        RevBulkData data3 = hubEx3.getBulkInputData();
        if(data3 == null) {
            slowUpdate();
            return;
        }

        int leftTopCount = data3.getMotorCurrentPosition(leftTop);
        int leftBottomCount = data3.getMotorCurrentPosition(leftBottom);
        int rightTopCount = data3.getMotorCurrentPosition(rightTop);
        int rightBottomCount = data3.getMotorCurrentPosition(rightBottom);

        RevBulkData data2 = hubEx2.getBulkInputData();

        int conveyerCount = data2.getMotorCurrentPosition(conveyer);
        int liftCount = data2.getMotorCurrentPosition(lift);
        int leftIntakeCount = data2.getMotorCurrentPosition(leftIntake);
        int rightIntakeCount = data2.getMotorCurrentPosition(rightIntake);
        RobotData.updateValues(leftTopCount, leftBottomCount, rightTopCount, rightBottomCount, conveyerCount, liftCount, leftIntakeCount, rightIntakeCount);
    }

    private synchronized void slowUpdate() {
        RobotData.updateValues(
            leftTop.getCurrentPosition(),
            leftBottom.getCurrentPosition(),
            rightTop.getCurrentPosition(),
            rightBottom.getCurrentPosition(),
            0,0,0,0);
    }
}
