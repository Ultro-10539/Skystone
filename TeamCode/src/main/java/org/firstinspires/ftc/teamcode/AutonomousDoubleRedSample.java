package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "AutonomousDoubleRedSample")
public class AutonomousDoubleRedSample extends AutonomousRedSampleFoundation {
    @Override
    public void run() {
        map.getRightAuto().setPosition(0.6);
        map.getRightFinger().setPosition(0.0);
        forwardRed();
        sampleFoundation();
        firstSample();
        secondSample();
    }

    private void firstSample() {
        //move forwardBlue and turn then drive to foundation then move back
        driver.move(Direction.FORWARD, 0.3, 7);
        driver.turn(0.5, -82);


        //line up with wall
        driver.move(Direction.FORWARD, 0.5, 25);
//        while(back.getDistance(DistanceUnit.CM) > 11){
//            driver.move(Direction.FORWARD, 0.4);
//        }
//        driver.move(Direction.FORWARD, 0);

        //drive to foundation
        switch (pos) {
            case LEFT_CORNER:
                driver.move(Direction.BACKWARD, 0.9, 73, true);
                 break;
            case MIDDLE:
                driver.move(Direction.BACKWARD, 0.9, 73, true);
                break;
            case RIGHT_CORNER:
            default:
                driver.move(Direction.BACKWARD, 0.9, 67, true);
        }
//        if (pos == Status.LEFT_CORNER){
//            driver.move(Direction.BACKWARD, 0.9, 55, true);
//        } else if(pos == Status.MIDDLE){
//            driver.move(Direction.BACKWARD, 0.9, 63, true);
//        } else {
//            driver.move(Direction.BACKWARD, 0.9, 71, true);
//        }

        //drop stone
        map.getLeftFinger().setPosition(0.6);
        sleep(500);
        map.getLeftFinger().setPosition(1.0);


        //dr
    }

    @Override
    public void secondSample() {
        //drive to next stpne
        if (pos == Status.RIGHT_CORNER){
            driver.move(Direction.FORWARD, 0.8, 28.5, true);
        } else if(pos == Status.MIDDLE){ ;
            driver.move(Direction.FORWARD, 0.8, 41.5, true);
        } else {
            driver.move(Direction.FORWARD, 0.8, 49, true);
        }

        driver.turn(0.5, 82);

        map.getLeftAuto().setPosition(1.0);
        map.getLeftFinger().setPosition(0.6);

        //Drive up to blocks
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 20 || colorRight.getDistance(DistanceUnit.CM) <= 20)){
            driver.move(Direction.BACKWARD, 0.45);
        }
        driver.move(Direction.BACKWARD, 0);

        //pick up blocks
        map.getLeftAuto().setPosition(1.0);
        map.getLeftFinger().setPosition(1.0);
        sleep(500);
        map.getLeftAuto().setPosition(0.6);
        sleep(500);

        driver.move(Direction.FORWARD, 0.9, 4);
        driver.turn(0.5, -82);

        if (pos == Status.RIGHT_CORNER){
            driver.move(Direction.BACKWARD, 0.8, 36, true);
        } else if(pos == Status.MIDDLE){ ;
            driver.move(Direction.BACKWARD, 0.8, 44, true);
        } else {
            driver.move(Direction.BACKWARD, 0.8, 52, true);
        }

        //drop stone
        map.getLeftFinger().setPosition(0.6);
        sleep(500);
        map.getLeftFinger().setPosition(1.0);

        driver.move(Direction.FORWARD, 0.8, 20, true);
        map.getRightFinger().setPosition(0.5);
        map.getRightAuto().setPosition(0);
    }
}