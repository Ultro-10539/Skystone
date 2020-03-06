package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.drive.Vector;

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
        driver.turn(0.7, -76);
        driver.move(Vector.from(-75, 0), 0.7, 0.77, -87.8);

        //90 - -87.8
        //180 - -178
        //270 - -266

        //        driver.moveUntil(Direction.BACKWARD, 0.3, data -> (data.getColorLeftDistance() <= 15 || data.getColorRightDistance() <= 15), true);
//        addData("val", map.getDistanceBack().getDistance(DistanceUnit.CM));
    }


}
