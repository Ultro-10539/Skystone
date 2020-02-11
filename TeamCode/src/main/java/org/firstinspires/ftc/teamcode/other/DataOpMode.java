package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.monitor.RobotData;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;

import java.util.Locale;

@Autonomous(name = "DATA VALUES", group = "")
public class DataOpMode extends AutoOpMode  {
    @Override
    public void setup(DeviceMap mapper) {
        mapper.setUpMotors(hardwareMap);
        mapper.setupServos(hardwareMap);
        mapper.setupSensors(hardwareMap);
        mapper.setUpImu(hardwareMap);

    }

    @Override
    public void beforeLoop() {
        DeviceMap map = DeviceMap.getInstance();

        telemetry.addLine(RobotData.hello());
        for(Servo servo : map.getServos()) {
            telemetry.addData("servo:", servo.getPosition());
        }
        for(ColorSensor colorSensor : map.getColorSensors()) {
            telemetry.addData("colorsensor dist: (r, g, b): ", String.format(Locale.ENGLISH, "%d, %d, %d", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
        }
        telemetry.update();
    }

    @Override
    public void run() {

    }
}
