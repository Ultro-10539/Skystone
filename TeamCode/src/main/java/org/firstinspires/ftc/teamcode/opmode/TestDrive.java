package org.firstinspires.ftc.teamcode.opmode;

public class TestDrive extends DriveOpMode {
    @Override
    public void loop() {
        driver.moveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y);
    }
}
