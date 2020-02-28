package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;

@Autonomous(name = "Test Mode")
public class testingOpmode extends AutoPart1 {
    @Override
    public void beforeLoop() {
    }

    @Override
    public void run() {
        testMethod();
    }

    private void testMethod() {

        driver.stopAndReset();
        //uses left arm
        map.getLeftAuto().setPosition(1.0);
        map.getLeftFinger().setPosition(1.0);
        sleep(500);
        map.getLeftAuto().setPosition(0.6);
        sleep(2000);

        //        driver.moveUntil(Direction.BACKWARD, 0.3, data -> (data.getColorLeftDistance() <= 15 || data.getColorRightDistance() <= 15), true);
//        addData("val", map.getDistanceBack().getDistance(DistanceUnit.CM));
    }


}
