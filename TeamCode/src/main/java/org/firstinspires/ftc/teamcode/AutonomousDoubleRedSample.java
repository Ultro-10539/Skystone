package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "AutonomousDoubleRedSample")
public class AutonomousDoubleRedSample extends AutoPart1 {
    @Override
    public void run() {
        forward();
        sampleFoundation();
        firstSample();
        secondSample();
    }



    public void firstSample() {
        //move forward and turn then drive to foundation then move back
        driver.move(Direction.FORWARD, 0.3, 5);
        driver.turn(0.5, -83);

        //line up with wall
        while(back.getDistance(DistanceUnit.CM) > 16){
            driver.move(Direction.FORWARD, 0.7);
        }
        driver.move(Direction.FORWARD, 0);

        //drive to foundation
        if (pos == Status.RIGHT_CORNER){
            driver.move(Direction.BACKWARD, 0.9, 60, true);
        } else if(pos == Status.MIDDLE){
            driver.move(Direction.BACKWARD, 0.9, 68, true);
        } else {
            driver.move(Direction.BACKWARD, 0.9, 76, true);
        }

        //drop stone
        driver.openLeftFinger();
        sleep(750);
        driver.closeLeftFinger();

        driver.move(Direction.FORWARD, 0.8, 44, true);


        //dr
    }
    protected void sampleFoundation() {
        //Line up with correct block
        if (pos == Status.RIGHT_CORNER){
            driver.move(Direction.RIGHT, 0.7, 8);
        } else if(pos == Status.MIDDLE){
            driver.move(Direction.RIGHT, 0.7, 14.5);
        } else {
            while(left.getDistance(DistanceUnit.CM) > 5){
                driver.move(Direction.RIGHT, 0.5);
            }
            driver.stop();
        }

        //Drive up to blocks
        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 20 || colorRight.getDistance(DistanceUnit.CM) <= 20)){
            driver.move(Direction.BACKWARD, 0.4);
        }
        driver.stop();

        //pick up blocks
        driver.closeLeftArm();
        driver.closeLeftFinger();
        sleep(500);
        driver.openLeftArm();
        sleep(500);


    }

    public void secondSample() {
        driver.turn(0.55, 83);
        Runnable prepareAction, pickUpAction, dropAction;
        //CLOSE = GRAB
        //OPEN = UNGRAB
        switch (pos) {
            case LEFT_CORNER:
                prepareAction = driver::prepareLeft;


                pickUpAction = () -> {
                    driver.closeLeftArm();
                    driver.closeLeftFinger();
                    sleep(500);
                    driver.openLeftArm();
                };

                dropAction = driver::openLeftFinger;

                break;
            case RIGHT_CORNER:
                driver.move(Direction.LEFT, 0.7, 7);
            case MIDDLE:
            default:
                prepareAction = driver::prepareRight;

                pickUpAction = () -> {
                    driver.closeRightArm();
                    driver.closeRightFinger();
                    sleep(500);
                    driver.openRightArm();
                };
                dropAction = driver::openRightFinger;
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
        sleep(250);

        //park
        driver.move(Direction.FORWARD, 0.8, 35, true);



    }
}