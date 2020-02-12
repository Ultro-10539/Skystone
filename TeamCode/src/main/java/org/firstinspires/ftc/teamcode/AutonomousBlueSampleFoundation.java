package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.monitor.RobotData;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "AutonomousBlueSampleFoundation")
public class AutonomousBlueSampleFoundation extends AutonomousBlueSample {
    private Status pos;
    @Override
    public void beforeLoop() {
        Status status = skystone();
        telemetry.addData("Status: ", status.name());
        updateTelemetry();
        pos = status;

        DeviceMap map = DeviceMap.getInstance();
        RevBlinkinLedDriver driver = map.getLedDriver();
        switch (pos) {
            case MIDDLE:
                driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case LEFT_CORNER:
                driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case RIGHT_CORNER:
                driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
            default:
                driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                break;
        }
    }

    @Override
    public void run() {
        sampleFoundation();
    }

    public void sampleFoundation() {
        DeviceMap map = DeviceMap.getInstance();
        driver.stopAndReset();
        //Prepares servo arms
        map.getRightAuto().setPosition(0.3);
        map.getRightFinger().setPosition(0.5);

        //Drives forwards a bit
        while(RobotData.distBack <= 100){
            driver.move(Direction.BACKWARD, 0.3);
        }
        driver.move(Direction.BACKWARD, 0);

        //Strafes to line up with wall
        while(RobotData.distRight > 5){
            driver.move(Direction.LEFT, 0.7);
        }
        driver.move(Direction.LEFT, 0);

        //Line up with correct block
        if (pos == Status.LEFT_CORNER){
            while(RobotData.distRight < 40){
                driver.move(Direction.RIGHT, 0.7);
            }
            driver.move(Direction.RIGHT, 0);
        } else if(pos == Status.MIDDLE){
            while(RobotData.distRight < 15){
                driver.move(Direction.RIGHT, 0.7);
            }
            driver.move(Direction.RIGHT, 0);
        }

        //Drive up to blocks
        while(!(RobotData.distColorLeft <= 20) || (RobotData.distColorLeft <= 20)){
            driver.move(Direction.BACKWARD, 0.3);
        }
        driver.move(Direction.BACKWARD, 0);

        //pick up blocks
        map.getRightAuto().setPosition(0.0);
        map.getRightFinger().setPosition(0.0);
        sleep(1000);
        map.getRightAuto().setPosition(0.6);

        //move forward and strafe to foundation then move back again
        driver.move(Direction.FORWARD, 0.3);
        //TODO: this is inconsistent
        sleep(400);
        driver.move(Direction.FORWARD, 0);
        if (pos == Status.LEFT_CORNER){
            driver.move(Direction.RIGHT, 0.7, 108);
        } else if(pos == Status.MIDDLE){
            driver.move(Direction.RIGHT, 0.7, 114);
        } else {
            driver.move(Direction.RIGHT, 0.7, 120);
        }
        while(!(RobotData.distColorLeft <= 20) || (RobotData.distColorLeft <= 20)){
            driver.move(Direction.BACKWARD, 0.3);
        }
        driver.move(Direction.BACKWARD, 0);

        //drop stone and grab foundation
        map.getRightFinger().setPosition(0.5);
        sleep(1000);
        map.getRightFinger().setPosition(0.0);
        sleep(1000);
        map.getFoundationLeft().setPosition(1.0);
        sleep(1000);

        //drive to wall then turn
        driver.move(Direction.FORWARD, 0.3);
        sleep(1000);
        driver.move(Direction.FORWARD, 0);
        driver.turn(0.7, 90);
    }
}