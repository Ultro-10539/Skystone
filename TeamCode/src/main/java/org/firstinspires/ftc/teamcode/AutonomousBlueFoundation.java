package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.Direction;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;

@Autonomous(name = "AutoBlueFoundation")
public class AutonomousBlueFoundation extends AutoPart1 {
    @Override
    public void run(){
        DeviceMap map = DeviceMap.getInstance();
        map.getLeftFinger().setPosition(1);
        map.getRightFinger().setPosition(0);
        driver.move(Direction.BACKWARD, 0.3, 30);
        map.getLeftFinger().setPosition(0);
        map.getRightFinger().setPosition(1);
        sleep(1000);
        driver.move(Direction.FORWARD, 0.7, 50);
        driver.turn(1.0, 90);
        map.getLeftFinger().setPosition(1);
        map.getRightFinger().setPosition(0);
        driver.move(Direction.RIGHT, 0.7, 20);
        driver.move(Direction.BACKWARD, 0.3, 10);
        driver.move(Direction.FORWARD, 0.7, 30);
    }

}
