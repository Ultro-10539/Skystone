package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.threading.Threader;

@TeleOp(name="Cartesian Driver op mode", group="Iterative Opmode")
public class TestDrive extends DriveOpMode {

    @Override
    public void init() {
        super.init();
        Threader.registerDrive();
    }
    @Override
    public void loop() {
        driver.moveFieldCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);
    }


    @Override
    public void stop() {
        Threader.destroy();
    }
}
