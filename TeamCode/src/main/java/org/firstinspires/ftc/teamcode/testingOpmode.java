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
        driver.moveUntil(Direction.BACKWARD, 0.3, data -> (data.getColorLeftDistance() <= 15 || data.getColorRightDistance() <= 15), true);
//        addData("val", map.getDistanceBack().getDistance(DistanceUnit.CM));
    }


}
