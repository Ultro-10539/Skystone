package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "AutonomousDoubleBlueSample")
public class AutonomousDoubleBlueSample extends AutonomousBlueSampleFoundation {
    @Override
    public void run() {
        forward();
        sampleFoundation();
        firstSample();
        secondSample();
    }

    @Override
    public void firstSample() {
        //move forward and turn then drive to foundation then move back
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

        driver.move(Direction.FORWARD, 0.8, 44, true);


        //dr
    }

    @Override
    public void secondSample() {
        driver.turn(0.55, -83);
        Servo auto, finger;
        double openAuto, openFinger,
                closeAuto, closeFinger,
                preparedAuto, preparedFinger;

        //CLOSE = GRAB
        //OPEN = UNGRAB
        switch (pos) {
            case LEFT_CORNER:
                auto = map.getLeftAuto();
                finger = map.getLeftFinger();
                openAuto = 1;
                openFinger = 1;
                preparedFinger = 0.5;
                preparedAuto = 0.3;
                closeAuto = 0;
                closeFinger = 0;
                break;
            case RIGHT_CORNER:
                driver.move(Direction.LEFT, 0.7, 7);
            case MIDDLE:
            default:
                auto = map.getRightAuto();
                finger = map.getRightFinger();
                openAuto = 0;
                openFinger = 0;
                preparedFinger = 0.5;
                preparedAuto = 0.3;
                closeAuto = 0.6;
                closeFinger = 1;
                break;
        }


        auto.setPosition(preparedAuto);
        finger.setPosition(preparedFinger);

        //Drive up to blocks
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 20 || colorRight.getDistance(DistanceUnit.CM) <= 20)){
            driver.move(Direction.BACKWARD, 0.45);
        }
        driver.move(Direction.BACKWARD, 0);

        auto.setPosition(closeAuto);
        finger.setPosition(closeFinger);
        sleep(500);
        auto.setPosition(openAuto);

        driver.move(Direction.FORWARD, 0.9, 4);
        driver.turn(0.5, -83);

        driver.move(Direction.BACKWARD, 0.9, 55, true);

        auto.setPosition(openAuto);
        finger.setPosition(openFinger);
        sleep(500);

        driver.move(Direction.FORWARD, 0.8, 35, true);



    }
}