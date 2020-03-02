package org.firstinspires.ftc.teamcode.threading.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import net.jafama.FastMath;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.threading.UltroThread;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicBoolean;

public class DriveThread extends UltroThread {
    private final MotorData[] data;
    private final Runnable clear;
    private volatile boolean awaiting;
    private final ExecutorService service = Executors.newSingleThreadScheduledExecutor();

    public DriveThread() {
        super();

        DeviceMap map = DeviceMap.getInstance();
        MotorData leftTop = new MotorData(map.getLeftTop());
        MotorData rightTop = new MotorData(map.getRightTop());
        MotorData leftBottom = new MotorData(map.getLeftBottom());
        MotorData rightBottom = new MotorData(map.getRightBottom());

        data = new MotorData[]{leftTop, rightTop, leftBottom, rightBottom};
        this.awaiting = false;
        clear = map::clearBulkCache;
    }

    @Override
    public void setUp(DeviceMap map) {
    }

    private class MotorData {
        private final DcMotor motor;
        private double power;
        private int pos;
        private double finalPos;
        /**
         * true = the power must be updated again
         * false = the power no longer needs updating
         */
        private AtomicBoolean needsUpdate;

        private MotorData(DcMotor motor) {
            this.motor = motor;
            needsUpdate = new AtomicBoolean(false);
        }

        @Override
        public String toString() {
            return "MotorData{" +
                    "motor=" + motor +
                    ", power=" + power +
                    ", pos=" + pos +
                    ", finalPos=" + finalPos +
                    ", needsUpdate=" + needsUpdate +
                    '}';
        }
    }

    @Override
    public void go() {
        clear.run();
        synchronized (data) {
            for(MotorData d : data) {
                //positions
                d.pos = d.motor.getCurrentPosition();
                boolean a = false;
                if(FastMath.abs(d.pos) >= FastMath.abs(d.finalPos)) {
                    //pos++
                    //final pos
                    d.power = 0;
                    a = true;
                    awaiting = false;
                }

                //if the motor positions aren't finish and it doesn't need a update, return.
                if(!a && !d.needsUpdate.get()) continue;
                d.motor.setPower(d.power);
                d.needsUpdate.set(false);
            }
        }
    }


    public void setPowers(double... powers) {
        synchronized (data) {
            for(int i = 0, size = data.length; i < size; i++) {
                double power = powers[i];
                MotorData d = data[i];
                if(d.power == power) continue;
                d.power = power;
                d.needsUpdate.set(true);
            }
        }
    }

    public int[] getCurrentPosition() {
        int size = data.length;
        int[] doubles = new int[size];
        for(int i = 0; i < size; i++) {
            doubles[i] = data[i].pos;
        }
        return doubles;
    }
    public void setPosition(int... positions) {
        synchronized (data) {
            awaiting = true;
            for(int i = 0, size = positions.length; i < size; i++) {
                data[i].finalPos = positions[i];
            }
        }
    }

    public void await() {
        final Telemetry telemetry = DeviceMap.getInstance().getTelemetry();
        final int TIMEOUT = 10;
        long startTime = System.currentTimeMillis();

        synchronized (data) {
            Future<Boolean> nothing =  service.submit(() -> {
                RobotLog.dd("ULTRO", "SUBMITTING");
                while(awaiting) {
                    telemetry.addLine("awaiting:" + awaiting);
                    if(System.currentTimeMillis() - startTime > TIMEOUT * 1000L) {
                        telemetry.addLine("time: " + System.currentTimeMillis());
                        telemetry.addLine("time: " + startTime);
                        awaiting  = false;
                        break;
                    }
                    telemetry.update();
                }
                return awaiting;
            });

            try {
                RobotLog.dd("ULTRO", "SUBMIT ONGOING" + System.currentTimeMillis());
                nothing.get();
                RobotLog.dd("ULTRO", "SUBMIT DONE" + System.currentTimeMillis());
            }catch (ExecutionException | InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}
