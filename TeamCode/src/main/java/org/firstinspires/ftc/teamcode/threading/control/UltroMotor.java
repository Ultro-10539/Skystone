package org.firstinspires.ftc.teamcode.threading.control;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.monitor.RobotData;
import org.firstinspires.ftc.teamcode.threading.UltroThread;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

public class UltroMotor extends UltroThread {
    private DcMotor leftTop, leftBottom, rightTop, rightBottom,
            leftIntake, rightIntake, conveyer;
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

        hubEx = map.getExpansionHub();
    }

    @Override
    public void run() {
        experimentalUpdate();
    }

    private void experimentalUpdate() {
        RevBulkData data = hubEx.getBulkInputData();
        RobotData.updateValues(
            data.getMotorCurrentPosition(leftTop),
            data.getMotorCurrentPosition(leftBottom),
            data.getMotorCurrentPosition(rightTop),
            data.getMotorCurrentPosition(rightBottom),
            data.getMotorCurrentPosition(leftIntake),
            data.getMotorCurrentPosition(rightIntake),
            data.getMotorCurrentPosition(conveyer));
    }

    private void slowUpdate() {
        RobotData.updateValues(
            leftTop.getCurrentPosition(),
            leftBottom.getCurrentPosition(),
            rightTop.getCurrentPosition(),
            rightBottom.getCurrentPosition(),
            leftIntake.getCurrentPosition(),
            rightIntake.getCurrentPosition(),
            conveyer.getCurrentPosition());
    }
}
