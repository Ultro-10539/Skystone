package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.firstinspires.ftc.teamcode.skystone.SkystonePipeline;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "Test Mode")
public class testingOpmode extends AutoPart1 {
    @Override
    public void beforeLoop() {
    }

    @Override
    public void run() {
        testMethod();
    }

    private void testMethod() {

        driver.stopAndReset();
        driver.moveUntil(Direction.FORWARD, 0.3, data -> data.getBackDistance() < 16);
    }


}
