package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;

@Autonomous(name = "AutonomousBlueDoubleSampleFoundation")
public class AutonomousBlueDoubleSampleFoundation extends AutoPart1 {
    @Override
    public void run() {
        forward();
        sampleFoundation();
        afterPickUp();
        secondSample();
    }

    protected void forward() {

        driver.stopAndReset();
        //Prepares servo arms
        map.getRightAuto().setPosition(0.3);
        map.getRightFinger().setPosition(0.5);


        //Drives forwards a bit
        driver.move(Direction.BACKWARD, 0.5, 21, true);
        driver.stop();
    }
    protected void sampleFoundation() {
        //colloquially: move the left direction with left power until the right sensor reads under 5 centimeters
        driver.moveUntil(Direction.LEFT, 0.7D, data -> data.getRightDistance() < 5);
        switch (pos) {
            case LEFT_CORNER:
                driver.moveUntil(Direction.RIGHT, 0.7D, data -> data.getRightDistance() > 40);
                break;
            case MIDDLE:
                driver.moveUntil(Direction.RIGHT, 0.7D, data -> data.getRightDistance() > 15);
                break;
            default:
                break;
        }

        //Drive up to blocks
        driver.moveUntil(Direction.BACKWARD, 0.3,
            data -> data.getColorLeftDistance() <= 20 ||
                    data.getColorRightDistance() <= 20);

        //pick up blocks
        map.getRightAuto().setPosition(0.0);
        map.getRightFinger().setPosition(0.0);
        sleep(500);
        map.getRightAuto().setPosition(0.6);
        sleep(500);


    }

    protected void afterPickUp() {

        //move forward and turn then drive to foundation then move back
        driver.move(Direction.FORWARD, 0.3, 5.75);
        driver.turn(0.5, 83);

        driver.moveUntil(Direction.FORWARD, 0.4, data -> data.getBackDistance() <= 16);

        //drive to foundation
        driver.move(Direction.BACKWARD, 0.7, 100, true);

        //face foundation
        driver.turn(0.5, -83);

        //line up with foundation

        driver.moveUntil(Direction.BACKWARD, 0.3,
                data -> data.getColorLeftDistance() <= 13 ||
                        data.getColorRightDistance() <= 13);

        //drop stone and grab foundation
        map.getRightFinger().setPosition(0.5);
        sleep(500);
        map.getRightFinger().setPosition(0.0);
        map.getFoundationLeft().setPosition(1.0);
        map.getFoundationRight().setPosition(0);
        sleep(750);

        //drive to wall then turn

        driver.move(Direction.FORWARD, 0.3, 40);
        driver.turn(0.7, 83);
        map.getFoundationLeft().setPosition(0);
        map.getFoundationRight().setPosition(1.0);
        sleep(750);
    }

        /**
         * Precondition: right is next to the wall.
         * 77 cm
         */
    protected void secondSample() {
        //strafe left
        driver.moveUntil(Direction.LEFT, 0.7, data -> right.getDistance(DistanceUnit.CM) < 77);
        //SecondSample
        //CHANGE 70

        //CONDITION HERE:
        int displacement;
        Servo auto, finger;
        //change these, just in case the servos have differing values.
        double autoOn, autoOff, fingerOn, fingerOff;
        switch (pos) {
            case LEFT_CORNER:
                auto = map.getLeftAuto();
                finger = map.getLeftFinger();
                displacement = 70;
                break;
            case RIGHT_CORNER:
                auto = map.getRightAuto();
                finger = map.getRightFinger();
                displacement = 70;
                break;
            case MIDDLE:
            default:
                auto = map.getLeftAuto();
                finger = map.getLeftFinger();
                displacement = 70;
                break;
        }

        //prepare the servo arms
        auto.setPosition(0.3);
        finger.setPosition(0.5);
        driver.move(Direction.FORWARD, 0.7, 70, true);
        //pray for the angle
        driver.turn(-83, 0.7);

        //Drive up to blocks
        driver.moveUntil(Direction.BACKWARD, 0.3,
                data -> data.getColorLeftDistance() <= 20 ||
                        data.getColorRightDistance() <= 20);

        //next to it, so pick up.
        finger.setPosition(1);
        sleep(650);
        //if you want accuracy, get the delta motor count from moving to the color sensor and multiply it by inches per counts and put it there.
        driver.move(Direction.FORWARD, 0.9, 2);
        driver.turn(0.7, 83);
        driver.move(Direction.BACKWARD, 0.8, 50, true);

        //let go
        finger.setPosition(0);
        sleep(250);
        driver.move(Direction.FORWARD, 0.7, 22);

        //if applicable, turn all the servos on to maximize parking space

    }
    private void correctLocation() {
        //driver.move(Direction.FORWARD, 0.7, RobotuData.distBack - 115, true);
        driver.move(Direction.FORWARD, 0.7);
        DistanceSensor left = map.getDistanceLeft();
        while (left.getDistance(DistanceUnit.CM) > 115) {

        }
        driver.stop();
    }

}