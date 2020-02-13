package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "AutonomousBlueSampleFoundation")
public class AutonomousRedSampleFoundation extends AutoPart1 {
    private DeviceMap map;
    private Status pos;



    private DistanceSensor left, back, right, colorLeft, colorRight;
    @Override
    public void beforeLoop() {
        map = DeviceMap.getInstance();

        left = map.getDistanceLeft();
        back = map.getDistanceBack();
        right = map.getDistanceRight();

        colorLeft = map.getSensorColorLeftDist();
        colorRight = map.getSensorColorRightDist();

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
        forward();
        sampleFoundation();
        afterPickUp();
    }

    private void forward() {

        driver.stopAndReset();
        //Prepares servo arms
        map.getRightAuto().setPosition(0.3);
        map.getRightFinger().setPosition(0.5);


        //Drives forwards a bit
        driver.move(Direction.BACKWARD, 0.5, 21, true);
        driver.stop();
    }
    public void sampleFoundation() {
        //Strafes to line up with wall
        while(left.getDistance(DistanceUnit.CM) > 5){
            driver.move(Direction.RIGHT, 0.7);
        }
        driver.stop();

        //Line up with correct block
        if (pos == Status.LEFT_CORNER){
            while(left.getDistance(DistanceUnit.CM) < 40){
                driver.move(Direction.LEFT, 0.7);
            }
            driver.stop();
        } else if(pos == Status.MIDDLE){
            while(left.getDistance(DistanceUnit.CM) < 15){
                driver.move(Direction.LEFT, 0.7);
            }
            driver.stop();
        }

        //Drive up to blocks
        boolean a = true;
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 20 || colorRight.getDistance(DistanceUnit.CM) <= 20)){
            driver.move(Direction.BACKWARD, 0.3);
        }


        driver.stop();

        //pick up blocks
        map.getLeftAuto().setPosition(0.0);
        map.getLeftFinger().setPosition(0.0);
        sleep(1000);
        map.getLeftAuto().setPosition(0.6);
        sleep(500);


    }

    private void afterPickUp() {

        //move forward and strafe to foundation then move back again
        driver.move(Direction.FORWARD, 0.3, 2.5);
        int distRight;
        switch (pos) {
            case LEFT_CORNER:
                distRight = 108;
                break;
            case MIDDLE:
                distRight = 114;
                break;
            case RIGHT_CORNER:
                distRight = 120;
                break;
            default:
                distRight = 50;

        }
        driver.move(Direction.LEFT, 0.7, distRight);
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 13 || colorRight.getDistance(DistanceUnit.CM) <= 13)){
            driver.move(Direction.BACKWARD, 0.3);
        }
        driver.move(Direction.BACKWARD, 0);

        //drop stone and grab foundation
        map.getLeftFinger().setPosition(0.5);
        sleep(1000);
        map.getLeftFinger().setPosition(0.0);
        sleep(1000);
        map.getFoundationLeft().setPosition(1.0);
        map.getFoundationRight().setPosition(0);
        sleep(1000);

        //drive to wall then turn
        driver.move(Direction.FORWARD, 0.3);
        sleep(1000);
        driver.move(Direction.FORWARD, 0);
        driver.turn(0.7, -90);
    }
    private void correctLocation() {
        //driver.move(Direction.FORWARD, 0.7, RobotData.distBack - 115, true);
        driver.move(Direction.FORWARD, 0.7);
        DistanceSensor left = map.getDistanceLeft();
        while (left.getDistance(DistanceUnit.CM) > 115) {

        }
        driver.stop();
    }
}