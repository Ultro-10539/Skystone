/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Button;
import org.firstinspires.ftc.teamcode.drive.MecanumDriver;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.opmode.DriveOpMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="Driver Button op mode", group="Iterative Opmode")
public class DriveButtonOpMode extends DriveOpMode {
    private List<Button> buttons;
    private DcMotor lift;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private boolean liftOverride = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    public String color = "red";

    @Override
    public void init() {
        super.init();

        DeviceMap mapper = DeviceMap.getInstance();
        addData("Status", "Initialized");
        updateTelemetry();

        this.lift = mapper.getLift();
        buttons = this.setUpButtons(mapper);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public List<Button> setUpButtons(DeviceMap mapper) {
        Button.Builder builder = new Button.Builder();
        List<Button> buttons = new ArrayList<>();

        Servo left = mapper.getLeftAuto();
        Servo right = mapper.getRightAuto();

        Servo leftPinch = mapper.getLeftFinger();
        Servo rightPinch = mapper.getRightFinger();

        buttons.addAll(Arrays.asList(
            builder.setGetter(() -> gamepad1.a)
                .setbFunction(() -> {
                    if(right.getPosition() >= 0.55){
                        right.setPosition(0.0);
                    } else {
                        right.setPosition(0.6);
                    }
                }).build(),

            builder.setGetter(() -> gamepad1.x)
                .setbFunction(() -> {
                    if(rightPinch.getPosition() >= 0.45){
                        rightPinch.setPosition(0.0);
                    } else {
                        rightPinch.setPosition(0.5);
                    }
                }).build(),

            builder.setGetter(() -> gamepad1.b)
                .setbFunction(() -> {
                    if(left.getPosition() <= 0.65){
                        left.setPosition(1.0);
                    } else {
                        left.setPosition(0.6);
                    }
                }).build(),
            builder.setGetter(() -> gamepad1.y)
                .setbFunction(() -> {
                    if(leftPinch.getPosition() <= 0.65){
                        leftPinch.setPosition(1.0);
                    } else {
                        leftPinch.setPosition(0.6);
                    }
                }).build(),
            builder.setGetter(() -> gamepad2.right_bumper)
                .setbFunction(() -> {
                    if (mapper.getClaw().getPosition() < 0.5){
                        mapper.getClaw().setPosition(1);
                    } else {
                        mapper.getClaw().setPosition(0);
                    }
                }).build(),
            builder.setGetter(() -> gamepad1.left_bumper)
                    .setbFunction(() -> {
                        if (color.equalsIgnoreCase("red")) {
                            color = "blue";
                            mapper.getLedDriver().setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE);
                        } else {
                            color = "red";
                            mapper.getLedDriver().setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
                        }
                    }).build(),
            builder.setGetter(() -> gamepad2.left_bumper)
                    .setbFunction(() -> {
                        if(mapper.getArm1().getPosition() < 0.5){
                            mapper.getArm1().setPosition(1);
                            mapper.getArm2().setPosition(1);
                        } else {
                            mapper.getArm1().setPosition(0);
                            mapper.getArm2().setPosition(0);
                        }
                    }).build(),
            builder.setGetter(() -> gamepad2.x)
                    .setbFunction(() -> {
                        if(!liftOverride){
                            liftOverride = true;
                        } else {
                            liftOverride = false;
                        }
                }).build(),
            builder.setGetter(() -> gamepad2.b)
                    .setbFunction(() -> {
                        if(mapper.getCap().getPosition() < 0.5) {
                            mapper.getCap().setPosition(1);
                        }else mapper.getCap().setPosition(0);
                    }).build(),

            builder.setGetter(() -> !gamepad2.dpad_up)
                .setbFunction(()-> {
                    mapper.getLift().setPower(0);
                }).build(),

            builder.setGetter(() -> !gamepad2.dpad_down)
                .setbFunction(()->{
                mapper.getLift().setPower(0);
            }).build()

            ));
        return buttons;
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        DeviceMap mapper = DeviceMap.getInstance();
        mapper.getRightAuto().setPosition(0.6);
        mapper.getRightFinger().setPosition(0.0);
        mapper.getClaw().setPosition(1);
        mapper.getArm1().setPosition(1);
        mapper.getArm2().setPosition(1);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        DeviceMap mapper = DeviceMap.getInstance();
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double right_stick_x = -gamepad1.right_stick_x;
        double multiplier = gamepad1.left_trigger + 1;
        if(color.equalsIgnoreCase("red"))
            driver.moveTrigRed(x / multiplier, y / multiplier, right_stick_x / multiplier);
        else driver.moveTrigBlue(x / multiplier, y / multiplier, right_stick_x / multiplier);
        driver.intake(-gamepad2.left_stick_y, -gamepad2.right_stick_y);

        driver.conveyer(gamepad2.left_trigger - gamepad2.right_trigger);

        if (gamepad1.right_trigger > 0) {
            mapper.getFoundationLeft().setPosition(1);
            mapper.getFoundationRight().setPosition(0);
        } else {
            mapper.getFoundationLeft().setPosition(0);
            mapper.getFoundationRight().setPosition(1);

        }


        double liftPos = lift.getCurrentPosition();

        if (gamepad2.dpad_up && liftPos < 7050 && !liftOverride) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(1);
        } else if (gamepad2.dpad_down && liftPos > 100 && !liftOverride ) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(-1);
        } else if (gamepad2.dpad_up && liftOverride) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(1);
        } else if (gamepad2.dpad_down && liftOverride) {
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(-1);
        }else if (gamepad2.a){
            lift.setTargetPosition(50);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(-1);
        }else if (gamepad2.y) {
            lift.setTargetPosition(2000);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
        }else if (!lift.isBusy()){
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(0);
        }

        addData("lift: ", liftPos);
        addData("Overrride: ", liftOverride);
        addData("Motor Status: ", lift.getMode());
        addData("trigger: ", gamepad2.left_trigger);

        for(Button button : buttons) button.press();
        updateTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    protected void addData(String header, Object value) {
        driver.addData(header, value);
    }
    protected void updateTelemetry() {
        driver.updateTelemetry();
    }

}
