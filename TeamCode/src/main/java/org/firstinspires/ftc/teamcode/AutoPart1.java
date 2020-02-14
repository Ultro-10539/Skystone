package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.firstinspires.ftc.teamcode.skystone.SkystonePipeline;
import org.firstinspires.ftc.teamcode.skystone.Status;

public abstract class AutoPart1 extends AutoOpMode {
    protected Status pos;
    protected DeviceMap map;
    protected DistanceSensor left, back, right, colorLeft, colorRight;


    protected SkystonePipeline pipeline;
    @Override
    public void setup(DeviceMap map) {

        map.setUpImu(hardwareMap);
        map.setUpMotors(hardwareMap);
        map.setupSensors(hardwareMap);
        map.setupServos(hardwareMap);
        map.setupOpenCV(hardwareMap);
        map.setUpLEDs(hardwareMap);

        int orient = hardwareMap.appContext.getResources().getConfiguration().orientation;
        map.getCamera().setPipeline(pipeline = new SkystonePipeline(orient, 640, 480));

        DeviceMap.getInstance().getCamera().startStreaming(pipeline.getRows(), pipeline.getCols());

        left = map.getDistanceLeft();
        back = map.getDistanceBack();
        right = map.getDistanceRight();

        colorLeft = map.getSensorColorLeftDist();
        colorRight = map.getSensorColorRightDist();

        this.map = map;
    }


    @Override
    public void beforeLoop() {

        Status status = skystone();
        telemetry.addData("Status: ", status.name());
        updateTelemetry();
        pos = status;

        RevBlinkinLedDriver driver = map.getLedDriver();
        switch (pos) {
            case MIDDLE:
                driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case LEFT_CORNER:
                driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
            case RIGHT_CORNER:
                driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
            default:
                driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
                break;
        }
    }

    protected Status skystone() {
        return pipeline.skystone();
    }



}
