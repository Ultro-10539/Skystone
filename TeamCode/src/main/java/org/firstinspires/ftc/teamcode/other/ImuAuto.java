package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.drive.Vector;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.firstinspires.ftc.teamcode.threading.Threader;
import org.firstinspires.ftc.teamcode.threading.control.UltroImu;

import java.util.Locale;

@Autonomous(name="IMU: help me", group="Linear Opmode")
public class ImuAuto extends AutoOpMode {
    private DcMotor motor;
    @Override
    public void setup(DeviceMap map) {
        map.setUpImu(hardwareMap);
        map.setUpDriveMotors(hardwareMap);
        map.initLynx(hardwareMap);
        telemetry.addData("Setted up imu!", "ready to go!");
        motor = map.getLeftBottom();
    }

    @Override
    public void beforeLoop() {
        telemetry.addLine(String.format(Locale.ENGLISH, "Angle: %f", Threader.get(UltroImu.class).getAngle()));
        telemetry.update();
    }

    @Override
    public void run() {
        driver.move(Vector.from(0, 100), 0.2, 0.75, 0);

    }
}
