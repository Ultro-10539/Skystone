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
    private ExpansionHubEx hubEx;
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

        hubEx = map.getExpansionHub();
    }

    @Override
    public void go() {
        experimentalUpdate();
    }

    private void experimentalUpdate() {
        if(hubEx == null) {
            slowUpdate();
            return;
        }
        RevBulkData data = hubEx.getBulkInputData();
        if(data == null) {
            slowUpdate();
            return;
        }

        RobotData.leftTop = data.getMotorCurrentPosition(leftTop);
        RobotData.leftBottom = data.getMotorCurrentPosition(leftBottom);
        RobotData.rightTop = data.getMotorCurrentPosition(rightTop);
        RobotData.rightBottom = data.getMotorCurrentPosition(rightBottom);
    }

    private synchronized void slowUpdate() {
        RobotData.updateValues(
            leftTop.getCurrentPosition(),
            leftBottom.getCurrentPosition(),
            rightTop.getCurrentPosition(),
            rightBottom.getCurrentPosition(),
            leftIntake.getCurrentPosition(),
            rightIntake.getCurrentPosition(),
            conveyer.getCurrentPosition(),
            lift.getCurrentPosition());
    }
}
