package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.firstinspires.ftc.teamcode.threading.Threader;
import org.firstinspires.ftc.teamcode.threading.control.UltroImu;

import java.util.Locale;

@Autonomous(name="IMU: help me", group="Linear Opmode")
public class ImuAuto extends AutoOpMode {
    @Override
    public void preInit() {
        super.preInit();
        //if you're pro, do this
        //driver.setTest(false);
        driver.setTest(true);
        //MonitorManager.startAll(DeviceMap.getInstance());
    }

    @Override
    public void setup(DeviceMap map) {
        map.setUpImu(hardwareMap);
        map.setUpMotors(hardwareMap);
        telemetry.addData("Setted up imu!", "ready to go!");
    }

    @Override
    public void beforeLoop() {
        telemetry.addLine(String.format(Locale.ENGLISH, "Angle: %f", Threader.get(UltroImu.class).getAngle()));
        telemetry.update();
    }

    @Override
    public void run() {
        driver.turn(0.65,83F);

        telemetry.addLine("Passed turning negative thrice");


    }
}
