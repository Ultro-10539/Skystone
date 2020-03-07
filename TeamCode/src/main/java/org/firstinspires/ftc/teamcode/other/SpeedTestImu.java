package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.RobotLog;

import net.jafama.FastMath;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.MecanumDriver;
import org.firstinspires.ftc.teamcode.drive.Vector;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.firstinspires.ftc.teamcode.threading.Threader;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Speed GyroDrive Test", group="Linear Opmode")
public class SpeedTestImu extends AutoOpMode {


    @Override
    public void preInit() {
        DeviceMap mapper = DeviceMap.getInstance(hardwareMap);
        mapper.setCurrentOpMode(this);
        mapper.setTelemetry(telemetry);
        setup(mapper);

        driver = new MecanumDriver();
        driver.setTelemetry(telemetry);

        addData("Status", "Initialized");

        updateTelemetry();
    }

    @Override
    public void setup(DeviceMap map) {
        map.setUpImu(hardwareMap);
    }

    @Override
    public void beforeLoop() {
        DeviceMap map = DeviceMap.getInstance();
        telemetry.addLine("imu power mode:" + map.getImu().getParameters().gyroPowerMode.toString());
        telemetry.addLine("imu mode:" + map.getImu().getParameters().mode.toString());
    }

    @Override
    public void run() {

        DeviceMap map = DeviceMap.getInstance();
        int TRIALS = 1000;
        List<Long> periods = new ArrayList<>();
        for(int i = 0; i < TRIALS; i++) {
            long start = System.currentTimeMillis();
            Orientation orientation = map.getImu().getAngularOrientation();
            long newTime = System.currentTimeMillis();
            periods.add(newTime - start);
        }
        double avg = 0;
        for(long period : periods) {
            telemetry.addLine("period: " + period);
            telemetry.update();
            avg += period;
        }

        avg = avg/(double) TRIALS;
        telemetry.addLine("average period (ms): " + avg);
        telemetry.addLine("average frequency(hz): " + (1D/avg) * 1000);

        telemetry.update();
        sleep(50000);

    }
}
