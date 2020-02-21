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

        //line up with wall
        while(back.getDistance(DistanceUnit.CM) > 16){
            driver.move(Direction.FORWARD, 0.7);
        }
        driver.move(Direction.FORWARD, 0);

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

        Runnable prepareAction, pickUpAction, dropAction;
        //CLOSE = GRAB
        //OPEN = UNGRAB
        switch (pos) {
            case RIGHT_CORNER:
                prepareAction = driver::prepareRight;

                pickUpAction = () -> {
                    driver.closeRightArm();
                    driver.closeRightFinger();
                    sleep(500);
                    driver.openRightArm();
                };
                dropAction = driver::openRightFinger;
                break;
            case LEFT_CORNER:
                driver.move(Direction.RIGHT, 0.7, 7);
            case MIDDLE:
            default:
                prepareAction = driver::prepareLeft;


                pickUpAction = () -> {
                    driver.closeLeftArm();
                    driver.closeLeftFinger();
                    sleep(500);
                    driver.openLeftArm();
                };

                dropAction = driver::openLeftFinger;
                break;
        }


        prepareAction.run();

        //Drive up to blocks
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 20 || colorRight.getDistance(DistanceUnit.CM) <= 20)){
            driver.move(Direction.BACKWARD, 0.45);
        }
        driver.move(Direction.BACKWARD, 0);

        pickUpAction.run();

        driver.move(Direction.FORWARD, 0.9, 4);
        driver.turn(0.5, -83);

        driver.move(Direction.BACKWARD, 0.9, 55, true);

        dropAction.run();
        sleep(500);

        //park
        driver.move(Direction.FORWARD, 0.8, 35, true);



    }
}