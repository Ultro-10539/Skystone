package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "🔴AutonomousRedSampleFoundation NOTIMER")
public class AutonomousRedSampleFoundation extends AutoPart1 {
    @Override
    public void run() {
        forwardRed();
        sampleStone();
//        firstSample();
//        secondSample();

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
        sleep(1000);
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
        sleep(1000);
        driver.move(Direction.FORWARD, 0.3, 5);
    }

    private void firstSample() {
        //move forward 5 inches and turn facing towards foundation
        driver.turn(0.5, 83.5);

        //drive until 120cm away from other wall
        //TODO: may need to adjust distance value
        driver.moveCond(Direction.FORWARD, 0.9, back.getDistance(DistanceUnit.CM) >= 120, true);


        //face foundation
        driver.turn(0.5, -83);

        //line up with foundation
        driver.moveCond(Direction.BACKWARD, 0.4, !(colorLeft.getDistance(DistanceUnit.CM) <= 13 || colorRight.getDistance(DistanceUnit.CM) <= 13), true);

        //grab foundation and drop stone
        map.getFoundationLeft().setPosition(1);
        map.getFoundationRight().setPosition(0);
        map.getLeftFinger().setPosition(0.6);
        map.getRightFinger().setPosition(0.5);
        sleep(500);
        map.getLeftFinger().setPosition(1.0);
        map.getRightFinger().setPosition(0.0);



        //drive toward wall a bit then turn
        //TODO: may need to adjust distance values.
        driver.moveCond(Direction.BACKWARD, 0.7, back.getDistance(DistanceUnit.CM) >= 120, true);
        driver.turn(0.7, -83);
        map.getFoundationLeft().setPosition(0);
        map.getFoundationRight().setPosition(1);
        //TODO: strafe left or right depending on distance sensor value
    }

    protected void secondSample() {
        //drive forward to next sample
        //TODO: change distance value so that robot is properly aligned to pick up left/middle
        driver.moveCond(Direction.BACKWARD, 0.8, back.getDistance(DistanceUnit.CM) >= 120, true);
        driver.turn(0.7, -83);

        //Line up with correct block
        //Directions are switched because robot is oriented differently on blue than on red
        sampleStone();
    }

    protected void foundation(){
        //move forward 5 inches and turn facing towards foundation
        driver.move(Direction.FORWARD, 0.3, 5);
        driver.turn(0.5, -83);

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