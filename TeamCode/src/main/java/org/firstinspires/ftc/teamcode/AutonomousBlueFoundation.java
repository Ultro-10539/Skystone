package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;

@Autonomous(name = "AutoBlueFoundation")
public class AutonomousBlueFoundation extends AutoPart1 {
    @Override
    public void run(){
        DeviceMap map = DeviceMap.getInstance();
        //strafe then move toward foundation
        driver.move(Direction.RIGHT, 0.7, 10);
        driver.move(Direction.BACKWARD, 0.3, 20);

        //drop stone and grab foundation
        map.getFoundationLeft().setPosition(1.0);
        sleep(1000);

        //drive to wall then turn
        while(map.getDistanceBack().getDistance(DistanceUnit.CM) > 5){
            driver.move(Direction.FORWARD, 0.3);
        }
        driver.move(Direction.FORWARD, 0);
        //turn needed
        //strafe away from wall and park
    }

}
