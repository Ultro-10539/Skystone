package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.firstinspires.ftc.teamcode.threading.Threader;
import org.firstinspires.ftc.teamcode.threading.control.UltroImu;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.Locale;

@Autonomous(name="SensorTest", group="Linear Opmode")
public class SensorTest extends AutoOpMode {
//    @Override
    public void preInit() {
        super.preInit();
        //if you're pro, do this
        //driver.setTest(false);

    }

    @Override
    public void beforeLoop() {

    }

    @Override
    public void run() {
        while(opModeisActive()) {
            ExpansionHubEx hub = DeviceMap.getInstance().getExpansionHub();
            RevBulkData bulkData = hub.getBulkInputData();
            for(DcMotor motor : driver.getMotors()) {
                telemetry.addLine("Motor Position: " + bulkData.getMotorCurrentPosition(motor));
            }

            UltroImu imu = Threader.get(UltroImu.class);
            double[] floats = new double[] {
                    imu.getYaw(),
                    imu.getRoll(),
                    imu.getPitch(),
            };
            telemetry.addLine(String.format(Locale.ENGLISH, "Angles: %f %f %f", floats[0], floats[1], floats[2]));


            telemetry.addData("Current I2C Bus Power Draw:", hub.getI2cBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.update();
        }
    }
}
