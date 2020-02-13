package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "AutonomousBlueSampleFoundation")
public class AutonomousBlueSampleFoundation extends AutoPart1 {
    @Override
    public void run() {
        forward();
        sampleFoundation();
        afterPickUp();
        park();
    }

    protected void forward() {

        driver.stopAndReset();
        //Prepares servo arms
        map.getRightAuto().setPosition(0.3);
        map.getRightFinger().setPosition(0.5);


        //Drives forwards a bit
        driver.move(Direction.BACKWARD, 0.5, 21, true);
        driver.stop();
    }
    protected void sampleFoundation() {
        //Strafes to line up with wall
        while(right.getDistance(DistanceUnit.CM) > 5){
            driver.move(Direction.LEFT, 0.7);
        }
        driver.move(Direction.LEFT, 0);

        //Line up with correct block
        if (pos == Status.LEFT_CORNER){
            while(right.getDistance(DistanceUnit.CM) < 40){
                driver.move(Direction.RIGHT, 0.7);
            }
            driver.move(Direction.RIGHT, 0);
        } else if(pos == Status.MIDDLE){
            while(right.getDistance(DistanceUnit.CM) < 17){
                driver.move(Direction.RIGHT, 0.7);
            }
            driver.move(Direction.RIGHT, 0);
        }

        //Drive up to blocks
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 20 || colorRight.getDistance(DistanceUnit.CM) <= 20)) {
            driver.move(Direction.BACKWARD, 0.3);
        }

            /*
        //colloquially: move the left direction with left power until the right sensor reads under 5 centimeters
        driver.moveUntil(Direction.LEFT, 0.7D, data -> data.getRightDistance() < 5);
        switch (pos) {
            case LEFT_CORNER:
                driver.moveUntil(Direction.RIGHT, 0.7D, data -> data.getRightDistance() > 40);
                break;
            case MIDDLE:
                driver.moveUntil(Direction.RIGHT, 0.7D, data -> data.getRightDistance() > 15);
                break;
            default:
                break;
        }

             */

        //Drive up to blocks
        driver.moveUntil(Direction.BACKWARD, 0.3,
            data -> data.getColorLeftDistance() <= 20 ||
                    data.getColorRightDistance() <= 20);

        //pick up blocks
        map.getRightAuto().setPosition(0.0);
        map.getRightFinger().setPosition(0.0);
        sleep(1000);
        map.getRightAuto().setPosition(0.6);
        sleep(500);


    }

    protected void afterPickUp() {

        //move forward and turn then drive to foundation then move back
        driver.move(Direction.FORWARD, 0.3, 5);
        driver.turn(0.5, 83);

        //line up with wall
        while(back.getDistance(DistanceUnit.CM) > 16){
            driver.move(Direction.FORWARD, 0.4);
        }
        driver.move(Direction.FORWARD, 0);

        //drive to foundation
        driver.move(Direction.BACKWARD, 0.7, 100, true);

        //face foundation
        driver.turn(0.5, -83);

        //line up with foundation
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 13 || colorRight.getDistance(DistanceUnit.CM) <= 13)){
            driver.move(Direction.BACKWARD, 0.3);
        }
        driver.move(Direction.BACKWARD, 0);

        //drop stone and grab foundation
        map.getRightFinger().setPosition(0.5);
        sleep(1000);
        map.getRightFinger().setPosition(0.0);
        map.getFoundationLeft().setPosition(1.0);
        map.getFoundationRight().setPosition(0);
        sleep(750);

        //drive to wall then turn

        driver.move(Direction.FORWARD, 0.3, 30);
        driver.turn(0.7, 83);
        driver.move(Direction.RIGHT, 0.8, 6);
        map.getFoundationLeft().setPosition(0);
        map.getFoundationRight().setPosition(1.0);
        sleep(750);
        driver.move(Direction.LEFT, 0.8, 9);
    }

        /**
         * Precondition: right is next to the wall.
         * 77 cm
         */
    protected void park() {
        //strafe left
        while(right.getDistance(DistanceUnit.CM) < 77){
            driver.move(Direction.LEFT, 0.7);
        }
        driver.move(Direction.LEFT, 0);
        //park
        driver.move(Direction.FORWARD, 0.7, 40);
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