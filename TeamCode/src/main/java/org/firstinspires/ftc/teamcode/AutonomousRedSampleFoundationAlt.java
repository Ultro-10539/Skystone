package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;

@Autonomous(name = "🔴SampleFirstFoundation")
public class AutonomousRedSampleFoundationAlt extends AutoPart1 {
    @Override
    public void run() {
        forwardRed();
        sampleStone();
        firstSample();
        secondSample();
        foundation();
    }


    protected void forwardRed() {

        driver.stopAndReset();
        map.getClaw().setPosition(1);
        switch(pos){
            case LEFT_CORNER:
                //prepares left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(0.6);
                driver.move(Direction.RIGHT, 0.7, 8);
                //closes right arm
                map.getRightAuto().setPosition(0.6);
                map.getRightFinger().setPosition(0.0);
                break;
            case MIDDLE:
                //prepares left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(0.6);

                //closes right arm
                map.getRightAuto().setPosition(0.6);
                map.getRightFinger().setPosition(0.0);
                break;
            case RIGHT_CORNER:
                //prepares right arm
                map.getRightAuto().setPosition(0.3);
                map.getRightFinger().setPosition(0.5);

                //closes left arm
                map.getLeftAuto().setPosition(0.6);
                map.getLeftFinger().setPosition(1.0);
                break;
            default:
                //prepares left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(0.6);

                //closes right arm
                map.getRightAuto().setPosition(0.6);
                map.getRightFinger().setPosition(0.0);
                break;
        }

        //Drives forwards a bit
        driver.move(Direction.BACKWARD, 0.7, 21, true);
    }

    protected void sampleStone() {
        //Drive up to blocks
        driver.moveUntil(Direction.BACKWARD, 0.3, data -> (data.getColorLeftDistance() <= 15 || data.getColorRightDistance() <= 15), true);

        //pick up blocks
        switch(pos){
            case LEFT_CORNER:
                //uses left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(1.0);
                sleep(500);
                map.getLeftAuto().setPosition(0.6);

                break;
            case MIDDLE:
                //uses left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(1.0);
                sleep(500);
                map.getLeftAuto().setPosition(0.6);

                break;
            case RIGHT_CORNER:
                //uses right arm
                map.getRightAuto().setPosition(0.0);
                map.getRightFinger().setPosition(0.0);
                sleep(500);
                map.getRightAuto().setPosition(0.6);

                break;
            default:
                //uses left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(1.0);
                sleep(500);
                map.getLeftAuto().setPosition(0.6);

                break;
        }
        driver.move(Direction.FORWARD, 0.3, 8);
    }

    private void firstSample() {
        //move forward 5 inches and turn facing towards foundation
        driver.turn(0.5, 83);

        //drive until 120cm away from other wall
        //TODO: may need to adjust distance value
        switch(pos){
            case LEFT_CORNER:
                driver.move(Direction.FORWARD, 0.9, 78, true);
                break;
            case MIDDLE:
                driver.move(Direction.FORWARD, 0.9, 70, true);
                break;
            case RIGHT_CORNER:
                driver.move(Direction.FORWARD, 0.9, 70, true);
                break;
            default:
                driver.move(Direction.FORWARD, 0.9, 70, true);
                break;
        }


        //face foundation
        driver.turn(0.5, -83);

        //line up with foundation
        driver.move(Direction.BACKWARD, 0.3, 9, true);

        //grab foundation and drop stone

        map.getLeftFinger().setPosition(0.6);
        map.getRightFinger().setPosition(0.5);
        driver.move(Direction.FORWARD, 0.3, 9, true);
        map.getLeftFinger().setPosition(1.0);
        map.getRightFinger().setPosition(0.0);
    }

    protected void secondSample() {
        driver.turn(0.3, -80);
        //TODO: change distance value so that robot is properly aligned to pick up left/middle
        driver.move(Direction.FORWARD, 0.9, 92, true);

        //Set up with correct block
        switch(pos){
            case LEFT_CORNER:
                //prepares left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(0.6);
                driver.move(Direction.RIGHT, 0.7, 8);
                //closes right arm
                map.getRightAuto().setPosition(0.6);
                map.getRightFinger().setPosition(0.0);
                break;
            case MIDDLE:
                //prepares left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(0.6);

                //closes right arm
                map.getRightAuto().setPosition(0.6);
                map.getRightFinger().setPosition(0.0);
                break;
            case RIGHT_CORNER:
                //prepares right arm
                map.getRightAuto().setPosition(0.3);
                map.getRightFinger().setPosition(0.5);

                //closes left arm
                map.getLeftAuto().setPosition(0.6);
                map.getLeftFinger().setPosition(1.0);
                break;
            default:
                //prepares left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(0.6);

                //closes right arm
                map.getRightAuto().setPosition(0.6);
                map.getRightFinger().setPosition(0.0);
                break;
        }
        driver.turn(0.5, 83);
        //Directions are switched because robot is oriented differently on blue than on red
        sampleStone();
        driver.turn(0.7, -76);
        //line up with wall
        driver.moveUntil(Direction.FORWARD, 0.7, data -> data.getBackDistance() <= 12, true);
    }

    protected void foundation(){
        //Drives toward foundation
        driver.move(Direction.BACKWARD, 0.9, 100, true);

        //face foundation
        driver.turn(0.7, 75);

        //line up with foundation
        driver.move(Direction.BACKWARD, 0.3, 10, false);

        //grab foundation and drop stone
        map.getLeftFinger().setPosition(0.6);
        map.getRightFinger().setPosition(0.5);
        driver.move(Direction.RIGHT, 0.7, 4, false);
        map.getFoundationLeft().setPosition(1);
        map.getFoundationRight().setPosition(0);
        sleep(1000);
        map.getLeftFinger().setPosition(1.0);
        map.getRightFinger().setPosition(0.0);
        //push foundation into correct position
        driver.turn(0.7, -74);
        map.getFoundationLeft().setPosition(0);
        map.getFoundationRight().setPosition(1);
        driver.move(Direction.BACKWARD, 0.9, 10, true);
        driver.move(Direction.FORWARD, 0.9, 45, true);
//
//        //park
//        driver.move(Direction.FORWARD, 0.7, 30, true);
//        map.getRightFinger().setPosition(0.5);
//        map.getRightAuto().setPosition(0);
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