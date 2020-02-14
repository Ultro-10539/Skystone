package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;

@Autonomous(name = "AutonomousBlueSample")
public class AutonomousBlueSample extends AutonomousBlueSampleFoundation {
    @Override
    public void run() {
        forward();
        sampleFoundation();
        firstSample();
    }

    @Override
    public void firstSample() {
        //move forward and turn then drive to foundation then move back
        driver.move(Direction.FORWARD, 0.3, 5);
        driver.turn(0.5, 83);

        //line up with wall
        while(back.getDistance(DistanceUnit.CM) > 16){
            driver.move(Direction.FORWARD, 0.4);
        }
        driver.move(Direction.FORWARD, 0);

        //drive to foundation
        driver.move(Direction.BACKWARD, 0.7, 85, true);
        driver.move(Direction.RIGHT, 0.7, 1);

        //drop stone
        map.getRightFinger().setPosition(0.5);
        sleep(750);
        map.getRightFinger().setPosition(0.0);
        driver.move(Direction.FORWARD, 0.7, 28, true);

        //dr
    }
}