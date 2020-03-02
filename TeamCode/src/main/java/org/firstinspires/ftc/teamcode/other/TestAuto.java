package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import net.jafama.FastMath;

import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.drive.Vector;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;

@Autonomous(name="Test Autonomous", group="Linear Opmode")
public class TestAuto extends AutoOpMode {
    @Override
    public void preInit() {
        super.preInit();
        //if you're pro, do this
        //driver.setTest(false);
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
        driver.move(Vector.from(-100, 0), 0.4, 0.1, -90);
    }
}
