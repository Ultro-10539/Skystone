package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "AutonomousDoubleBlueSample")
public class AutonomousDoubleBlueSample extends AutonomousBlueSampleFoundation {
    @Override
    public void run() {
        map.getRightAuto().setPosition(0.6);
        map.getRightFinger().setPosition(0.0);
        forwardBlue();
        sampleFoundation();
        firstSample();
        secondSample();
    }

    @Override
    public void firstSample() {
        //move forwardBlue and turn then drive to foundation then move back
        driver.move(Direction.FORWARD, 0.3, 5);
        driver.turn(0.5, 83);


//        //line up with wall
//        while(back.getDistance(DistanceUnit.CM) > 16){
//            driver.move(Direction.FORWARD, 0.4);
//        }
//        driver.move(Direction.FORWARD, 0);

        //drive to foundation
        if (pos == Status.LEFT_CORNER){
            driver.move(Direction.BACKWARD, 0.9, 60, true);
        } else if(pos == Status.MIDDLE){
            driver.move(Direction.BACKWARD, 0.9, 68, true);
        } else {
            driver.move(Direction.BACKWARD, 0.9, 76, true);
        }

        //drop stone
        map.getRightFinger().setPosition(0.5);
        sleep(750);
        map.getRightFinger().setPosition(0.0);


        //dr
    }

    @Override
    public void secondSample() {

        //drive to next stpne
        if (pos == Status.LEFT_CORNER){
            driver.move(Direction.FORWARD, 0.8, 36, true);
        } else if(pos == Status.MIDDLE){ ;
            driver.move(Direction.FORWARD, 0.8, 44, true);
        } else {
            driver.move(Direction.FORWARD, 0.8, 58, true);
        }

        map.getRightAuto().setPosition(0.3);
        map.getRightFinger().setPosition(0.5);

        driver.turn(0.5, -83);


        //Drive up to blocks
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 20 || colorRight.getDistance(DistanceUnit.CM) <= 20)){
            driver.move(Direction.BACKWARD, 0.45);
        }
        driver.move(Direction.BACKWARD, 0);

        //pick up blocks
        map.getRightAuto().setPosition(0.0);
        map.getRightFinger().setPosition(0.0);
        sleep(500);
        map.getRightAuto().setPosition(0.6);
        sleep(500);

        driver.move(Direction.FORWARD, 0.9, 4);
        driver.turn(0.5, 83);

        if (pos == Status.LEFT_CORNER){
            driver.move(Direction.BACKWARD, 0.8, 36, true);
        } else if(pos == Status.MIDDLE){ ;
            driver.move(Direction.BACKWARD, 0.8, 44, true);
        } else {
            driver.move(Direction.BACKWARD, 0.8, 52, true);
        }

        //drop stone
        map.getRightFinger().setPosition(0.5);
        sleep(750);
        map.getRightFinger().setPosition(0.0);

        driver.move(Direction.FORWARD, 0.8, 19, true);
        map.getRightFinger().setPosition(0.5);
        map.getRightAuto().setPosition(0);


    }
}