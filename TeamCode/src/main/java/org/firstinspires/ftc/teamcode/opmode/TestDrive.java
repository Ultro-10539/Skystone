package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Button;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.threading.Threader;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Cartesian Driver op mode", group="Iterative Opmode")
public class TestDrive extends DriveOpMode {
    @Override
    protected void afterInit(DeviceMap mapper) {
        Threader.registerDrive();
    }

    @Override
    protected List<Button> setUpButtons(Button.Builder builder, DeviceMap mapper) {
        return new ArrayList<>();
    }

    @Override
    protected void doWhileInitLoop(DeviceMap mapper) {

    }

    @Override
    protected void play(DeviceMap mapper) {

    }

    @Override
    protected void doWhile(DeviceMap mapper) {
        driver.moveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.right_stick_y);
    }

    @Override
    protected void end(DeviceMap mapper) {

    }

    @Override
    public void stop() {
        Threader.destroy();
    }
}
