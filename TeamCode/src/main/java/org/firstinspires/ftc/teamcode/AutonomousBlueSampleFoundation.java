package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;

@Autonomous(name = "AutonomousBlueSampleFoundation NOTIMER")
public class AutonomousBlueSampleFoundation extends AutoPart1 {
    @Override
    public void run() {
        forwardRed();
        sampleStone();
        firstSample();
        secondSample();

//        foundation();
    }

    protected void forwardRed() {

        driver.stopAndReset();
        map.getClaw().setPosition(1);


        //Drives forwards a bit
        driver.move(Direction.BACKWARD, 0.7, 21, true);
        driver.stop();
    }

    protected void sampleStone() {
        //Prepares servos and lines up with correct block
        //Prepares servo arms based on detected block
        switch(pos){
            case LEFT_CORNER:
                //prepares left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(0.6);
                driver.move(Direction.LEFT, 0.7, 8);
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

            default:
                //prepares left arm
                map.getLeftAuto().setPosition(1.0);
                map.getLeftFinger().setPosition(0.6);

                //closes right arm
                map.getRightAuto().setPosition(0.6);
                map.getRightFinger().setPosition(0.0);
                break;
        }

        //Drive up to blocks
        driver.moveCond(Direction.BACKWARD, 0.4, !(colorLeft.getDistance(DistanceUnit.CM) <= 20 || colorRight.getDistance(DistanceUnit.CM) <= 20), true);
//        while(!(colorLeft.getDistance(DistanceUnit.CM) <= 20 || colorRight.getDistance(DistanceUnit.CM) <= 20)){
//            driver.move(Direction.BACKWARD, 0.4);
//        }
//        driver.move(Direction.BACKWARD, 0);

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
        //move forward 5 inches and turn facing towards foundation
        driver.move(Direction.FORWARD, 0.3, 5);
        driver.turn(0.5, 83);

        //brings right Auto down because stone might block distance sensor
        map.getRightAuto().setPosition(0.3);
    }

    private void firstSample() {
        //drive until 120cm away from other wall
        //TODO: may need to adjust distance value
        driver.moveCond(Direction.BACKWARD, 0.9, back.getDistance(DistanceUnit.CM) >= 120, true);

        //brings right Auto back up
        map.getRightAuto().setPosition(0.6);

        //face foundation
        driver.turn(0.5, -83);

        //line up with foundation
        driver.moveCond(Direction.BACKWARD, 0.4, !(colorLeft.getDistance(DistanceUnit.CM) <= 13 || colorRight.getDistance(DistanceUnit.CM) <= 13), true);

        //drop stone
        map.getLeftFinger().setPosition(0.6);
        map.getRightFinger().setPosition(0.5);
        sleep(500);
        map.getLeftFinger().setPosition(1.0);
        map.getRightFinger().setPosition(0.0);

        //drive forward, turn 180, then grab foundation
        driver.move(Direction.FORWARD, 0.5, 5, true);
        driver.turn(0.5, -166);
        driver.move(Direction.FORWARD, 0.5, 5, true);
        map.getFoundationLeft().setPosition(1);
        map.getFoundationRight().setPosition(0);

        //drive toward wall a bit then turn
        //TODO: may need to adjust distance values.
        driver.moveCond(Direction.BACKWARD, 0.7, back.getDistance(DistanceUnit.CM) >= 120, true);
        driver.turn(0.7, 83);
        map.getFoundationLeft().setPosition(0);
        map.getFoundationRight().setPosition(1);
        //TODO: strafe left or right depending on distance sensor value
    }

    protected void secondSample() {
        //drive forward to next sample
        //TODO: change distance value so that robot is properly aligned to pick up left/middle
        driver.moveCond(Direction.BACKWARD, 0.8, back.getDistance(DistanceUnit.CM) >= 120, true);
        driver.turn(0.7, 83);

        //Line up with correct block
        //Directions are switched because robot is oriented differently on blue than on red
        sampleStone();
        map.getRightAuto().setPosition(0.6);
    }

    protected void foundation(){
        //drives until lined up with foundation
        driver.moveCond(Direction.BACKWARD, 0.4, !(colorLeft.getDistance(DistanceUnit.CM) <= 13 || colorRight.getDistance(DistanceUnit.CM) <= 13), true);

        //drop stone
        map.getLeftFinger().setPosition(0.6);
        map.getRightFinger().setPosition(0.5);
        sleep(500);
        map.getLeftFinger().setPosition(1.0);
        map.getRightFinger().setPosition(0.0);

        //push foundation into correct position
        driver.move(Direction.BACKWARD, 0.7, 10, true);

        //park
        driver.move(Direction.FORWARD, 0.7, 30, true);
        map.getRightFinger().setPosition(0.5);
        map.getRightAuto().setPosition(0);
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