package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "AutonomousRedSampleFoundation")
public class AutonomousRedSampleFoundation extends AutoPart1 {
    @Override
    public void run() {
        forward();
        sampleFoundation();
        afterPickUp();
    }
    public void sampleFoundation() {
        //Strafes to line up with wall
        while(left.getDistance(DistanceUnit.CM) > 7){
            driver.move(Direction.RIGHT, 0.7);
        }
        driver.move(Direction.RIGHT, 0);

        //Line up with correct block
        if (pos == Status.LEFT_CORNER){
            while(right.getDistance(DistanceUnit.CM) < 42){
                driver.move(Direction.LEFT, 0.7);
            }
            driver.move(Direction.LEFT, 0);
        } else if(pos == Status.MIDDLE){
            while(right.getDistance(DistanceUnit.CM) < 19){
                driver.move(Direction.LEFT, 0.7);
            }
            driver.move(Direction.LEFT, 0);
        }

        //Drive up to blocks
        boolean a = true;
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 20 || colorRight.getDistance(DistanceUnit.CM) <= 20)){
            driver.move(Direction.BACKWARD, 0.3);
        }


        driver.move(Direction.BACKWARD, 0);

        //pick up blocks
        map.getLeftAuto().setPosition(1.0);
        map.getLeftFinger().setPosition(1.0);
        sleep(1000);
        map.getLeftAuto().setPosition(0.6);
        sleep(500);


    }

    private void afterPickUp() {

        //move forward and turn then drive to foundation then move back
        driver.move(Direction.FORWARD, 0.3, 2.5);
        driver.turn(0.5, -83);

        //line up with wall
        while(back.getDistance(DistanceUnit.CM) > 16){
            driver.move(Direction.FORWARD, 0.4);
        }
        driver.move(Direction.FORWARD, 0);

        //drive to foundation
        driver.move(Direction.BACKWARD, 0.7, 100, true);

        //face foundation
        driver.turn(0.5, 83);

        //line up with foundation
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 13 || colorRight.getDistance(DistanceUnit.CM) <= 13)){
            driver.move(Direction.BACKWARD, 0.3);
        }
        driver.move(Direction.BACKWARD, 0);

        //drop stone and grab foundation
        map.getLeftFinger().setPosition(0.6);
        sleep(1000);
        map.getLeftFinger().setPosition(1.0);
        map.getFoundationLeft().setPosition(1.0);
        map.getFoundationRight().setPosition(0);
        sleep(750);

        //drive to wall then turn

        driver.move(Direction.FORWARD, 0.3, 30);
        driver.turn(0.7, -83);
        map.getFoundationLeft().setPosition(0);
        map.getFoundationRight().setPosition(1.0);
        //strafe right
        while(left.getDistance(DistanceUnit.CM) < 75){
            driver.move(Direction.RIGHT, 0.7);
        }
        driver.move(Direction.RIGHT, 0);
        //secondSample
        driver.move(Direction.FORWARD, 0.7, 55);
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