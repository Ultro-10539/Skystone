package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.skystone.Status;

@Autonomous(name = "AutonomousBlueSampleFoundation")
public class AutonomousBlueSampleFoundation extends AutonomousBlueSample {
    private Status pos;
    @Override
    public void beforeLoop() {
        Status status = skystone();
        telemetry.addData("Status: ", status.name());
        updateTelemetry();
        pos = status;
    }

    @Override
    public void run() {
        sampleFoundation();
    }

    public void sampleFoundation() {
        DeviceMap map = DeviceMap.getInstance();
        driver.stopAndReset();
        //Prepares servo arms
        map.getRightAuto().setPosition(0.3);
        map.getRightFinger().setPosition(0.5);

        //Drives forwards a bit
        while(map.getDistanceBack().getDistance(DistanceUnit.CM) <= 100){
            driver.move(Direction.BACKWARD, 0.3);
        }
        driver.move(Direction.BACKWARD, 0);

        //Strafes to line up with wall
        while(map.getDistanceRight().getDistance(DistanceUnit.CM) > 5){
            driver.move(Direction.LEFT, 0.7);
        }
        driver.move(Direction.LEFT, 0);

        //Line up with correct block
        if (pos == Status.LEFT_CORNER){
            while(map.getDistanceRight().getDistance(DistanceUnit.CM) < 40){
                driver.move(Direction.RIGHT, 0.7);
            }
            driver.move(Direction.RIGHT, 0);
        } else if(pos == Status.MIDDLE){
            while(map.getDistanceRight().getDistance(DistanceUnit.CM) < 15){
                driver.move(Direction.RIGHT, 0.7);
            }
            driver.move(Direction.RIGHT, 0);
        }

        //Drive up to blocks
        while(!(map.getSensorColorLeftDist().getDistance(DistanceUnit.CM) <= 20) || (map.getSensorColorRightDist().getDistance(DistanceUnit.CM) <= 20)){
            driver.move(Direction.BACKWARD, 0.3);
        }
        driver.move(Direction.BACKWARD, 0);

        //pick up blocks
        map.getRightAuto().setPosition(0.0);
        map.getRightFinger().setPosition(0.0);
        sleep(1000);
        map.getRightAuto().setPosition(0.6);

        //move forward and strafe to foundation then move back again
        driver.move(Direction.FORWARD, 0.3);
        sleep(400);
        driver.move(Direction.FORWARD, 0);
        if (pos == Status.LEFT_CORNER){
            driver.move(Direction.RIGHT, 0.7, 108);
        } else if(pos == Status.MIDDLE){
            driver.move(Direction.RIGHT, 0.7, 114);
        } else {
            driver.move(Direction.RIGHT, 0.7, 120);
        }
        while(!(map.getSensorColorLeftDist().getDistance(DistanceUnit.CM) <= 20) || (map.getSensorColorRightDist().getDistance(DistanceUnit.CM) <= 20)){
            driver.move(Direction.BACKWARD, 0.3);
        }
        driver.move(Direction.BACKWARD, 0);

        //drop stone and grab foundation
        map.getRightFinger().setPosition(0.5);
        sleep(1000);
        map.getRightFinger().setPosition(0.0);
        sleep(1000);
        map.getFoundation().setPosition(1.0);
        sleep(1000);

        //drive to wall then turn
        driver.move(Direction.FORWARD, 0.3);
        sleep(1000);
        driver.move(Direction.FORWARD, 0);
        driver.turn(0.7, 90);


    }
}