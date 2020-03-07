package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import net.jafama.FastMath;

import org.firstinspires.ftc.teamcode.drive.Vector;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;

@Autonomous(name="GyroDrive Test", group="Linear Opmode")
public class SpeedTestImu extends AutoOpMode {
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
        map.setUpDriveMotors(hardwareMap);
        map.setUpImu(hardwareMap);
        map.initLynx(hardwareMap);
    }

    @Override
    public void beforeLoop() {

    }

    @Override
    public void run() {

        final double dist = 2;
        for(int i = 0; i <= 360; i += 30) {
            double angleRadians = Math.toRadians(i);
            double x = dist * FastMath.sin(angleRadians);
            double y = dist * FastMath.cos(angleRadians);

            RobotLog.dd("ULTRO", "angle: " + i + " x: " + x +" y: " + y);
            driver.move(Vector.from(x, y), 1, 0.5, 0);
        }

        driver.stopAndReset();



        /*
        driver.move(Vector.from(0, 8), 0.35);
        sleep(3000);
        driver.move(Vector.from(8, 0), 0.35);

         */
    }
}
