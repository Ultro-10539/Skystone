package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.drive.Vector;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;

@Autonomous(name = "🔵DoubleSampleFoundation")
public class AutonomousBlueSampleFoundation extends AutoPart1 {
    @Override
    public void run() {
        DeviceMap mapper = DeviceMap.getInstance();
        mapper.getArm1().setPosition(1);
        mapper.getArm2().setPosition(1);
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
                driver.prepareLeft();
                driver.move(Direction.RIGHT, 0.7, 8);
                //closes right arm
                driver.closeRight();
                break;
            case RIGHT_CORNER:
                //prepares right arm
                driver.prepareRight();

                //closes left arm
                driver.closeLeft();
                break;
            case MIDDLE:
            default:
                //prepares left arm
                driver.prepareLeft();

                //closes right arm
                driver.closeRight();
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
        driver.move(Direction.FORWARD, 0.3, 7);
    }

    private void firstSample() {
        //move forward 5 inches and turn facing towards foundation
        driver.turn(0.7, -42);
        //89.3125
        //drive until 120cm away from other wall
        //TODO: may need to adjust distance value
        telemetry.addData("yes", "yes");
        switch(pos){
            case RIGHT_CORNER:
                System.out.println("test" + pos);
                driver.move(Vector.from(15, 0), 0.9, 0.5, -88.25);
                break;
            case MIDDLE:
                System.out.println("test" + pos);
                //67
                driver.move(Vector.from(15, 0), 0.9, 0.5, -88.25);
                break;
            case LEFT_CORNER:
                System.out.println("test" + pos);
                driver.move(Vector.from(15, 0), 0.9, 0.5, -88.25);
                break;
            default:
                System.out.println("test" + pos);
                driver.move(Vector.from(15, 0), 0.9, 0.5, -88.25);
                break;
        }

        driver.move(Vector.from(32.5, 0), 0.8, 0.7, 0);
        driver.stopAndReset();

        //face foundation
        //driver.turn(0.5, -83);

        //line up with foundation
        driver.moveUntil(Direction.BACKWARD, 0.3, data -> (data.getColorLeftDistance() <= 15 && data.getColorRightDistance() <= 15), true);

        //grab foundation and drop stone
        map.getFoundationLeft().setPosition(1);
        map.getFoundationRight().setPosition(0);
        map.getLeftFinger().setPosition(0.6);
        map.getRightFinger().setPosition(0.5);
        sleep(1000);
        map.getLeftFinger().setPosition(1.0);
        map.getRightFinger().setPosition(0.0);
        //push foundation into correct position
        driver.move(Direction.FORWARD, 0.9, 10, true);
        driver.turn(0.7, 73);
        map.getFoundationLeft().setPosition(0);
        map.getFoundationRight().setPosition(1);
        driver.move(Direction.RIGHT, 0.7, 2);
        driver.move(Vector.from(-77.5, 0), 0.9, 0.45, 92.7);
    }

    protected void secondSample() {
        //faces stones
        driver.turn(0.5, -84);

        //prepares stones
        switch(pos){
            case LEFT_CORNER:
                //prepares left arm
                map.getLeftAuto().setPosition(0.9);
                map.getLeftFinger().setPosition(0.6);

                //closes right arm
                map.getRightAuto().setPosition(0.6);
                map.getRightFinger().setPosition(0.0);
                break;
            case MIDDLE:
                //prepares right arm
                map.getRightAuto().setPosition(0.3);
                map.getRightFinger().setPosition(0.5);
//                driver.move(Direction.LEFT, 0.7, 8);

                //closes left arm
                map.getLeftAuto().setPosition(0.6);
                map.getLeftFinger().setPosition(1.0);
                break;
            case RIGHT_CORNER:
                //prepares right arm
                map.getRightAuto().setPosition(0.3);
                map.getRightFinger().setPosition(0.5);
                driver.move(Direction.LEFT, 0.7, 8);

                //closes left arm
                map.getLeftAuto().setPosition(0.6);
                map.getLeftFinger().setPosition(1.0);
                break;
            default:
                //prepares left arm
                map.getLeftAuto().setPosition(0.9);
                map.getLeftFinger().setPosition(0.6);

                //closes right arm
                map.getRightAuto().setPosition(0.6);
                map.getRightFinger().setPosition(0.0);
                break;
        }
        //samples stones
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
                //uses right arm
                map.getRightAuto().setPosition(0.0);
                map.getRightFinger().setPosition(0.0);
                sleep(500);
                map.getRightAuto().setPosition(0.6);

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
        driver.move(Direction.FORWARD, 0.3, 6);
    }

    protected void foundation(){
        //Drives toward foundation
        driver.turn(0.7, 76);
        driver.move(Vector.from(59, 0), 0.9, 0.45, 91.8);
        driver.moveUntil(Direction.BACKWARD, 0.3, data -> (data.getColorLeftDistance() <= 15 || data.getColorRightDistance() <= 15), true);
        //grab foundation and drop stone
        map.getLeftFinger().setPosition(0.6);
        map.getRightFinger().setPosition(0.5);
        sleep(1000);
        map.getLeftFinger().setPosition(1.0);
        map.getRightFinger().setPosition(0.0);
        //push foundation into correct position and park
        driver.move(Direction.BACKWARD, 0.9, 10, true);
        driver.move(Direction.FORWARD, 0.9, 45, true);
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