package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import net.jafama.FastMath;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Predicate;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.monitor.RobotData;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.firstinspires.ftc.teamcode.threading.Threader;
import org.firstinspires.ftc.teamcode.threading.control.DriveThread;
import org.firstinspires.ftc.teamcode.threading.control.UltroImu;

import java.util.Arrays;
import java.util.function.Consumer;

public final class MecanumDriver implements IDriver {
    private static final double TURN_OFFSET = 2.5F;
    private static final double MIN_POWER = 0.35D;

    private boolean test;
    private DeviceMap map;
    private Telemetry telemetry;
    private DcMotor[] motors;

    private boolean conveyerCurrent;

    private static final double COUNTS_PER_MOTOR_REV = 560;
    private static final double WHEEL_DIAMETER_INCHES   = 4.0;
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER_INCHES * FastMath.PI);

    public MecanumDriver() {
        this(false);
        this.map = DeviceMap.getInstance();
        this.motors = map.getDriveMotors();
    }

    public MecanumDriver(boolean test) {
        this.test = test;
    }

    public class Data {
        public DistanceSensor left, back, right, colorLeft, colorRight;
        public UltroImu imu;

        private Data() {
            left = map.getDistanceLeft();
            back = map.getDistanceBack();
            right = map.getDistanceRight();

            colorLeft = map.getSensorColorLeftDist();
            colorRight = map.getSensorColorRightDist();

            this.imu = Threader.get(UltroImu.class);
        }

        public double getLeftDistance() {
            return left.getDistance(DistanceUnit.CM);
        }

        public double getBackDistance() {
            return back.getDistance(DistanceUnit.CM);
        }

        public double getRightDistance() {
            return right.getDistance(DistanceUnit.CM);
        }

        public double getColorLeftDistance() {
            return colorLeft.getDistance(DistanceUnit.CM);
        }

        public double getColorRightDistance() {
            return colorRight.getDistance(DistanceUnit.CM);
        }

        public double getAngle() {
            return imu.getAngle();
        }
    }

    public void append(Direction direction, double power) {
        DcMotor leftTop = map.getLeftTop();
        DcMotor rightTop = map.getRightTop();
        DcMotor leftBottom = map.getLeftBottom();
        DcMotor rightBottom = map.getRightBottom();

        leftTop.setPower(leftTop.getPower() + direction.getLeftTop() * power);
        rightTop.setPower(rightTop.getPower() + direction.getRightTop() * power);
        leftBottom.setPower(leftBottom.getPower() + direction.getLeftBottom() * power);
        rightBottom.setPower(rightBottom.getPower() + direction.getRightBottom() * power);
    }

    public void moveUntil(Direction direction, double power, Predicate<Data> dataPredicate) {
        moveUntil(direction, power, dataPredicate, false);
    }
    /**
     * Move until a specified condition is reached
     * @param direction
     * @param power
     * @param dataPredicate if this returns true, then the robot will stop.
     *                      The data class has all the sensors necessary to make easy access of sensors possible.
     */
    public void moveUntil(Direction direction, double power, Predicate<Data> dataPredicate, boolean gyroAssist) {
        double angle = 0, initialAngle = 0;
        if(gyroAssist) {
            UltroImu imu = Threader.get(UltroImu.class);
            imu.resetAngle();
            angle = imu.getAngle();
            initialAngle = angle;
        }

        move(direction, power);
        Data data = new Data();
        //it is with an exclamation point for human readable logic
        final long TIMEOUT = 5000L;
        final long startTime = System.currentTimeMillis();
        while(!dataPredicate.test(data)) {
            long deltaTime = System.currentTimeMillis() - startTime;
            if(deltaTime > TIMEOUT)
                return;
            if (gyroAssist){
                gyroAssistor(direction, initialAngle, angle, power);
                UltroImu imu = Threader.get(UltroImu.class);
                angle = imu.getAngle();
            }
        }
        stopAndReset();
    }
    /**
     * Drive until a conditional is true
     * @param direction
     * @param power
     * @param conditional
     * @param gyroAssist
     */
    public void moveCond(Direction direction, double power, boolean conditional, boolean gyroAssist){
        double angle = 0, initialAngle = 0;
        if(gyroAssist) {
            UltroImu imu = Threader.get(UltroImu.class);
            imu.resetAngle();
            angle = imu.getAngle();
            initialAngle = angle;
        }
        LinearOpMode linear = null;

        while (linear.opModeIsActive() && conditional){
            move(direction, power);
            if (gyroAssist){
                gyroAssistor(direction, initialAngle, angle, power);
                UltroImu imu = Threader.get(UltroImu.class);
                angle = imu.getAngle();
            }
        }
        move(direction, 0);
    }

    public void stopAndReset(){
        stop();
        reset();
    }

    public void reset() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Move an unspecified amount of distance
     * @param direction
     * @param power
     */
    @Override
    public void move(Direction direction, double power) {
        move(direction, power, power, power, power);
    }

    public void move(Direction direction, double leftTop, double rightTop, double leftBottom, double rightBottom) {
        if(test) return;
        map.getLeftTop().setPower(leftTop * direction.getLeftTop());
        map.getRightTop().setPower(rightTop * direction.getRightTop());
        map.getLeftBottom().setPower(leftBottom * direction.getLeftBottom());
        map.getRightBottom().setPower(rightBottom * direction.getRightBottom());
    }

    @Override
    public void move(Direction direction, double power, double inches) {
        move(direction, power, inches, false);
    }

    public void move(Vector vector, double power, double turnSpeed, double preferredAngleDegrees) {
        this.move(vector, power, turnSpeed, preferredAngleDegrees, null);
    }

    /**
     * vectorized move function
     * @param vector
     * @param power
     */
    public void move(Vector vector, double power, double turnSpeed, double preferredAngleDegrees, Predicate<Data> dataPredicate) {

        double distanceToTarget = vector.length();
        double angle2Target = FastMath.atan2(vector.getY(), vector.getX());

        double relativeAngle;

        //encoder things
        this.clearBulkCache();
        int[] current = getMotorCounts();
        int[] old = Arrays.copyOf(current, current.length);


        int leftTopTarget = Integer.MAX_VALUE;
        int rightTopTarget = Integer.MAX_VALUE;
        int leftBottomTarget = Integer.MAX_VALUE;
        int rightBottomTarget = Integer.MAX_VALUE;
        boolean last = true;
        System.out.println("Vector: " + vector);


        double angle2TargetDegrees = FastMath.toDegrees(angle2Target);
        double prefferedRadians = FastMath.toRadians(preferredAngleDegrees);
        Data data = new Data();
        while(motorsBusy(leftTopTarget, rightTopTarget, leftBottomTarget, rightBottomTarget) ||
                (dataPredicate != null && !dataPredicate.test(data))) {
            double currentAngle = getAngle();
            relativeAngle = MathUtil.wrapAngle(angle2TargetDegrees - currentAngle);

            double relativeAngleRadians = FastMath.toRadians(relativeAngle);
            Vector relativeVector = Vector.from(
                    distanceToTarget * FastMath.cos(relativeAngleRadians),
                    distanceToTarget * FastMath.sin(relativeAngleRadians));

            System.out.println("Relative Vector: " + relativeVector);
            Vector normalized = relativeVector.normalize();

            double movementX = normalized.getX();
            double movementY = normalized.getY();


            double calcY = relativeVector.getY() * COUNTS_PER_INCH;
            double calcX = relativeVector.getX() * COUNTS_PER_INCH;
            leftTopTarget = FastMath.abs(old[1] + (int) (0.5 + (calcY + calcX)));
            rightTopTarget = FastMath.abs(old[2] + (int) (0.5 + (calcY - calcX)));
            leftBottomTarget = FastMath.abs(old[2] + (int) (0.5 + (calcY - calcX)));
            rightBottomTarget = FastMath.abs(old[3] + (int) (0.5 + (calcY + calcX)));
            //System.out.println("Encoder clicks: " + leftTopTarget + ", " + rightTopTarget + ", " + leftBottomTarget + ", " + rightBottomTarget);

            //power things
            double leftTopPower = power * (movementY + movementX);
            double rightTopPower = power * (movementY - movementX);
            double leftBottomPower = power * (movementY - movementX);
            double rightBottomPower = power * (movementY + movementX);
            //System.out.println("Powers (before turning) clicks: " + leftTopPower + ", " + rightTopPower);
            //System.out.println("Powers (before turning) clicks: " + leftBottomPower + ", " + rightBottomPower);

            //fix the angle

            double turnAngle =  FastMath.toRadians(currentAngle) - prefferedRadians;


            System.out.println("Current angle: " + currentAngle + " Relative angle: " + relativeAngle + " Turn Angle: " + FastMath.toDegrees(turnAngle));
            //telemetry.addLine("Current angle: " + FastMath.toDegrees(currentAngle) + " Relative angle: " + FastMath.toDegrees(relativeAngle) + " Turn Angle: " + turnAngle);

            double turnPower = Range.clip(turnAngle/FastMath.toRadians(20), -1, 1) * turnSpeed;

            leftTopPower = leftTopPower - turnPower;
            leftBottomPower = leftBottomPower - turnPower;

            rightTopPower = rightTopPower + turnPower;
            rightBottomPower = rightBottomPower + turnPower;

            double[] powers = reduce(new double[]{leftTopPower, rightTopPower, leftBottomPower, rightBottomPower});


            System.out.println("Turning (after turning) clicks: -" + turnPower +  ", +" + turnPower);
            System.out.println("Turning (after turning) clicks: -" + turnPower + ", +" + turnPower);

            System.out.println("Powers (after turning) clicks: " + powers[0] +  ", " + powers[1]);
            System.out.println("Powers (after turning) clicks: " + powers[2] + ", " + powers[3]);

/*
            telemetry.addLine("Powers (after turning) clicks: " + powers[0] + ", " + powers[1]);
            telemetry.addLine("Powers (after turning) clicks: " + powers[2] + ", " + powers[3]);
            telemetry.update();
 */
            System.out.println('\n');


            move(Direction.FORWARD, powers[0], powers[1], powers[2], powers[3]);


            boolean now = FastMath.abs(turnPower) > 0.09;
            if(last && !now) {
                int[] current2 = getMotorCountsCached();

                int[] deltaCount = new int[]{
                        FastMath.abs(current2[0] - current[0]),
                        FastMath.abs(current2[1] - current[1]),
                        FastMath.abs(current2[2] - current[2]),
                        FastMath.abs(current2[3] - current[3]),
                };
                current = current2;
                System.out.println("motor counts:" + Arrays.toString(deltaCount));
            }

            last = now;
        }
    }

    private double yesAngle = -80;
    public double getAngle() {
        if(test) {
            if (yesAngle >= -120) yesAngle -= 0.005;
            return yesAngle;
        }else {
            UltroImu imu = Threader.get(UltroImu.class);
            return imu.getGlobalAngle();
        }
    }

    private double[] reduce(double[] powers) {
        //short check first
        int j = 0;
        for(double power : powers) {
            if(FastMath.abs(power) > 1) j++;
        }
        if(j == 0) return powers;
        double maxPower = powers[0];
        for(int i = 1, length = powers.length; i < length; i++) {
            if(FastMath.abs(powers[i]) > FastMath.abs(maxPower)) maxPower = powers[i];
        }
        maxPower = FastMath.abs(maxPower);
        return new double[] {powers[0]/maxPower, powers[1]/maxPower, powers[2]/maxPower, powers[3]/maxPower};
    }


    public void await() {
        getDriveThread().await();
    }
    /**
     * encoder drive
     * @param direction
     * @param power
     * @param inches
     */
    public void move(Direction direction, double power, double inches, boolean gyroAssist) {
        /*
        if((direction == Direction.LEFT) || (direction == Direction.RIGHT)){
            inches *= (1D / 0.7D);
        }else if(direction == Direction.FORWARD || direction == Direction.BACKWARD) {
         */
        if(direction == Direction.BACKWARD || direction == Direction.FORWARD) inches -= 3.5D;
        //}

        double calc = COUNTS_PER_INCH * inches;

        int[] current = getMotorCounts();
        //other calculations needed
        int leftTopTarget = FastMath.abs(current[0] + (int) (calc * direction.getLeftTop()));
        int rightTopTarget = FastMath.abs(current[1] + (int) (calc * direction.getRightTop()));
        int leftBottomTarget = FastMath.abs(current[2] + (int) (calc * direction.getLeftBottom()));
        int rightBottomTarget = FastMath.abs(current[3] + (int) (calc * direction.getRightBottom()));

        //getDriveThread().setPosition(leftTopTarget, rightTopTarget, leftBottomTarget, rightBottomTarget);
        double angle = 0, initialAngle = 0;
        if(gyroAssist) {
            UltroImu imu = Threader.get(UltroImu.class);
            imu.resetAngle();
            angle = imu.getAngle();
            initialAngle = angle;
        }
        LinearOpMode linear = null;
        if(map.getCurrentOpMode() instanceof AutoOpMode) {
            linear = (LinearOpMode) map.getCurrentOpMode();
            if(linear == null) return;
        }
        move(direction, power);

        while(linear.opModeIsActive() && motorsBusy(leftTopTarget, rightTopTarget, leftBottomTarget, rightBottomTarget)) {
            if (!gyroAssist) continue;
            gyroAssistor(direction, initialAngle, angle, power);
            UltroImu imu = Threader.get(UltroImu.class);
            angle = imu.getAngle();
        }

        stopAndReset();

    }

    private void gyroAssistor(Direction direction, double initialAngle, double angle, double power) {
        double[] newPowers = new double[] {power, power, power, power};
        double correctedPower = power * calculatePowerMultiplierLinear(0, angle, power);
        switch (direction) {
            case FORWARD:
            case BACKWARD:
                if(angle > initialAngle + 2) {
                    //POWER UP RIGHT SIDE
                    for(int id : direction.getRightSide()) {
                        newPowers[id] = power + 0.1D;
                    }
                    for(int id : direction.getLeftSide()) {
                        newPowers[id] = correctedPower;
                    }
                }else if(angle < initialAngle - 2) {
                    //POWER UP LEFT SIDE
                    for(int id : direction.getRightSide()) {
                        newPowers[id] = correctedPower;
                    }
                    for(int id : direction.getLeftSide()) {
                        newPowers[id] = power + 0.1;
                    }
                }
                break;
                /*
            case LEFT:

                if(angle > initialAngle + 2) {
                    newPowers[0] = power - 0.1D;
                    newPowers[1] = power + 0.1D;
                    newPowers[2] = correctedPower;
                    newPowers[3] = -correctedPower;
                }else if(angle < initialAngle - 2) {
                    newPowers[3] = power - 0.1D;
                    newPowers[2] = power + 0.1D;
                    newPowers[1] = correctedPower;
                    newPowers[0] = -correctedPower;
                }
                break;
            case RIGHT:
                if(angle > initialAngle + 2) {
                    newPowers[3] = correctedPower;
                    newPowers[2] = -correctedPower;
                    newPowers[1] = power - 0.1D;
                    newPowers[0] = power + 0.1D;
                }else if(angle < initialAngle - 2) {
                    newPowers[0] = correctedPower;
                    newPowers[1] = -correctedPower;
                    newPowers[2] = power - 0.1D;
                    newPowers[3] = power + 0.1D;
                }
                break;

                 */
        }

        move(direction, newPowers[0], newPowers[1], newPowers[2], newPowers[3]);
    }
    /**
     * encoder drive
     * @param direction
     * @param inches
     */
    public void move(Direction direction, double inches, boolean gyroAssist) {
        /*
        if((direction == Direction.LEFT) || (direction == Direction.RIGHT)){
            inches *= (1D / 0.7D);
        }else if(direction == Direction.FORWARD || direction == Direction.BACKWARD) {
         */
        inches -= 3.5D;
        //}

        double calc = COUNTS_PER_INCH * inches;


        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        int[] current = getMotorCounts();
        //other calculations needed
        int leftTopTarget = FastMath.abs(current[0] + (int) (calc * direction.getLeftTop()));
        int rightTopTarget = FastMath.abs(current[1] + (int) (calc * direction.getRightTop()));
        int leftBottomTarget = FastMath.abs(current[2] + (int) (calc * direction.getLeftBottom()));
        int rightBottomTarget = FastMath.abs(current[3] + (int) (calc * direction.getRightBottom()));

        double angle = 0, initialAngle = 0;
        if(gyroAssist) {
            UltroImu imu = Threader.get(UltroImu.class);
            imu.resetAngle();
            angle = imu.getAngle();
            initialAngle = angle;
        }
        LinearOpMode linear = null;
        if(map.getCurrentOpMode() instanceof AutoOpMode) {
            linear = (LinearOpMode) map.getCurrentOpMode();
            if(linear == null) return;
        }
        double power;
        while(linear.opModeIsActive() && motorsBusy(leftTopTarget, rightTopTarget, leftBottomTarget, rightBottomTarget)) {
            int[] counts = getMotorCounts();
            double leftTopPower = findPower(counts[0], leftTopTarget);
            double rightTopPower = findPower(counts[1], rightTopTarget);
            double leftBottomPower = findPower(counts[2], leftBottomTarget);
            double rightBottomPower = findPower(counts[3], rightBottomTarget);
            power = (leftTopPower + rightTopPower + leftBottomPower + rightBottomPower) / 4D;
            if (!gyroAssist) continue;
            double correctedPower = power * calculatePowerMultiplierLinear(0, angle, power);
            addData("initial angle", initialAngle);
            addData("angle", angle);
            addData("power", power);
            if (angle > initialAngle + 2) {
                addData("Increasing right side", correctedPower);
                gyroAssist(direction.getRightSide(), power + 0.1);
                gyroAssist(direction.getLeftSide(), correctedPower);
            } else if(angle < initialAngle - 2) {
                addData("Increasing left side", correctedPower);
                gyroAssist(direction.getLeftSide(), power + 0.1);
                gyroAssist(direction.getRightSide(), correctedPower);
            }else {
                addData("normal", "side");
                move(direction, power);
            }
            updateTelemetry();

            UltroImu imu = Threader.get(UltroImu.class);
            angle = imu.getAngle();
        }

        stop();

        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private double findPower(double current, double finalCount) {
        final double MAX_POWER = .95D;

        return MAX_POWER - (current/finalCount);
    }

    private void gyroAssist(int[] side, double power) {
        for(int index : side) {
            DcMotor motor = map.getDriveMotors()[index];
            motor.setPower(power);
        }
    }

    /**
     *
     * @param MAX_POWER
     * @param leftTopTarget
     * @param rightTopTarget
     * @param leftBottomTarget
     * @param rightBottomTarget
     * @return
     */
    private double[] calculatePowerFromMotor(final double MAX_POWER, int leftTopTarget, int rightTopTarget, int leftBottomTarget, int rightBottomTarget) {
        int[] currents = getMotorCounts();
        int[] futures = new int[] {leftTopTarget, rightTopTarget, leftBottomTarget, rightBottomTarget};
        int size = currents.length;
        double[] powers = new double[size];
        for(int i = 0; i < size; i++) {
            double current = (double) currents[i];
            double average = (current + (double) futures[i])/2D;

            double power = -FastMath.pow2(current - average) + 1.2D;

            if(power > MAX_POWER) power = MAX_POWER;
            else if(power < MIN_POWER) power = MIN_POWER;
            powers[i] = power;
        }

        return powers;
    }

    /**
     * See: https://www.desmos.com/calculator/tugir8g8tf
     * @param expectedAngle
     * @param currentAngle
     * @param defaultPower
     * @return
     */
    private double calculatePowerMultiplier(double expectedAngle, double currentAngle, double defaultPower) {
        return defaultPower * (1D - 0.1D * FastMath.ceilToInt(FastMath.abs(expectedAngle - currentAngle)));
    }
    private double calculatePowerMultiplierLinear(double expectedAngle, double currentAngle, double defaultPower) {
        return defaultPower * (1D - 0.1D * FastMath.abs(expectedAngle - currentAngle));
    }

    public void move(AngleConverter angleConverter) {
        map.getLeftTop().setPower(angleConverter.getLeftTop());
        map.getRightTop().setPower(angleConverter.getRightTop());
        map.getRightBottom().setPower(angleConverter.getRightBottom());
        map.getLeftBottom().setPower(angleConverter.getLeftBottom());
    }

    public boolean motorsBusy(int leftTopTarget, int rightTopTarget, int leftBottomTarget, int rightBottomTarget) {
        int[] current = getMotorCounts();
        telemetry.addLine(Arrays.toString(current));
        telemetry.update();
        int[] target = new int[] { leftTopTarget, rightTopTarget, leftBottomTarget, rightBottomTarget };
        for(int i = 0; i < current.length; i++) {
            //1: current, target: 3 => current < target
            //-1: current, target: -10 => c
            if(FastMath.abs(current[i]) < target[i]) return true;
        }
        return false;
    }


    private int[] testCounts = new int[] {0, 0, 0, 0};

    public int[] getMotorCounts() {
        if(test) {
            for(int i = 0; i < testCounts.length; i++) {
                testCounts[i]++;
            }
            return testCounts;
        }
        this.clearBulkCache();
        return new int[] {map.getLeftTop().getCurrentPosition(), map.getRightTop().getCurrentPosition(), map.getLeftBottom().getCurrentPosition(), map.getRightBottom().getCurrentPosition()};
    }
    public int[] getMotorCountsCached() {
        if(test) return getMotorCounts();
        return new int[] {map.getLeftTop().getCurrentPosition(), map.getRightTop().getCurrentPosition(), map.getLeftBottom().getCurrentPosition(), map.getRightBottom().getCurrentPosition()};
    }
    /**
     * Updated mecanum drive function this year (math is ? ?? ? )
     * @param left_stick_x gamepadleftX
     * @param left_stick_y gamepadleftY
     * @param right_stick_x gamepadrightX
     */
    public void move(double left_stick_x, double left_stick_y, double right_stick_x){
        double LF = Range.clip(left_stick_y + left_stick_x + right_stick_x, -1, 1);
        double RF = Range.clip(left_stick_y - left_stick_x - right_stick_x, -1, 1);
        double LB = Range.clip(left_stick_y - left_stick_x + right_stick_x, -1, 1);
        double RB = Range.clip(left_stick_y + left_stick_x - right_stick_x, -1, 1);

        map.getLeftTop().setPower(LF);
        map.getRightTop().setPower(RF);
        map.getLeftBottom().setPower(LB);
        map.getRightBottom().setPower(RB);
    }


    public void moveFieldCentric(double left_stickX, double left_stickY, double right_stickX, double right_stickY) {

        //linear
        double r = FastMath.hypot(left_stickX, left_stickY);
        double angle = FastMath.atan2(left_stickY, left_stickX);


        double robotAngle = getAngle();
        double relativeAngle = angle - FastMath.toRadians(robotAngle);

        Vector vector = new Vector(r * FastMath.cos(relativeAngle), r * FastMath.sin(relativeAngle));


        System.out.println(vector);
        double movementX = vector.getX();
        double movementY = vector.getY();

        //rotation
        double preferredAngle = FastMath.atan2(right_stickY, right_stickX);
        double turnR = FastMath.hypot(right_stickX, right_stickY);
        double turnAngle =  FastMath.toRadians(robotAngle) - preferredAngle - FastMath.toRadians(90);

        double turnPower = Range.clip(turnAngle/Math.toRadians(30), -1, 1) * turnR;
        //power things
        double leftTopPower = (movementY + movementX) - turnPower;
        double leftBottomPower = (movementY - movementX) - turnPower;

        double rightTopPower = (movementY - movementX) + turnPower;
        double rightBottomPower = (movementY + movementX) + turnPower;

        double[] powers = reduce(new double[]{leftTopPower, rightTopPower, leftBottomPower, rightBottomPower});
        System.out.println("Current angle: " + robotAngle);
        System.out.println("powers: " + "[ " + powers[0] + " " + powers[1] +  " ]");
        System.out.println("powers: " + "[ " + powers[2] + " " + powers[3] +  " ]");
        move(Direction.FORWARD, powers[0], powers[1], powers[2], powers[3]);
    }
    /**
     * Updated mecanum drive function this year (math is ? ?? ? )
     * @param left_stick_x gamepadleftX
     * @param left_stick_y gamepadleftY
     * @param right_stick_x gamepadrightX
     */
    public void moveTrigRed(double left_stick_x, double left_stick_y, double right_stick_x){
        // how much to amplify the power
        double r = FastMath.hypot(left_stick_y, left_stick_x);

        //calculates the angle of the joystick - 45 degrees
        double robotAngle = FastMath.atan2(left_stick_y, left_stick_x) - FastMath.PI / 4;

        // rotation
        double rightX = right_stick_x;

        // equation for each of the wheels
        final double LF = r * FastMath.cos(robotAngle) + rightX;
        final double RF = r * FastMath.sin(robotAngle) - rightX;
        final double LB = r * FastMath.sin(robotAngle) + rightX;
        final double RB = r * FastMath.cos(robotAngle) - rightX;

        map.getLeftTop().setPower(LF);
        map.getRightTop().setPower(RF);
        map.getLeftBottom().setPower(LB);
        map.getRightBottom().setPower(RB);
    }

    /**
     * Updated mecanum drive function this year (math is ? ?? ? )
     * @param left_stick_x gamepadleftX
     * @param left_stick_y gamepadleftY
     * @param right_stick_x gamepadrightX
     */
    public void moveTrigBlue(double left_stick_x, double left_stick_y, double right_stick_x){
        // how much to amplify the power
        double r = FastMath.hypot(left_stick_y, left_stick_x);

        //calculates the angle of the joystick - 45 degrees
        double robotAngle = FastMath.atan2(left_stick_y, left_stick_x) - ((5*Math.PI) / 4);

        // rotation
        double rightX = right_stick_x;

        // equation for each of the wheels
        final double LF = r * FastMath.cos(robotAngle) + rightX;
        final double RF = r * FastMath.sin(robotAngle) - rightX;
        final double LB = r * FastMath.sin(robotAngle) + rightX;
        final double RB = r * FastMath.cos(robotAngle) - rightX;

        map.getLeftTop().setPower(LF);
        map.getRightTop().setPower(RF);
        map.getLeftBottom().setPower(LB);
        map.getRightBottom().setPower(RB);
    }

    /**
     *
     * @param power
     * @param angle; -90 is clockwise, 90 is counterclockwise
     */
    @Override
    public void turn(double power, double angle) {
        if(FastMath.abs(angle) > 180) {
            //if it's more than +180 or less than -180, add towards 0: 180
            if(angle > 180) turn(power, angle - 180);
            else if(angle < -180) turn(power, angle + 180);
            return;
        }
        Direction direction = angle > 0 ? Direction.CLOCKWISE : Direction.COUNTERCLOCKWISE;

        //angle = 50
        //turn_offset = 5

        //min = 45
        double min = angle - TURN_OFFSET;
        //min = 55
        double max = angle + TURN_OFFSET;


        UltroImu imu = Threader.get(UltroImu.class);
        double currentAngle = imu.getAngle();
        double firstAngle = currentAngle;

        min = min + firstAngle;
        max = max + firstAngle;

        DeviceMap map = DeviceMap.getInstance();
        LinearOpMode linear = null;
        if(map.getCurrentOpMode() instanceof AutoOpMode) {
            linear = (LinearOpMode) map.getCurrentOpMode();
        }


        move(direction, power);

        while ((linear != null && linear.opModeIsActive()) && !(min <= currentAngle && currentAngle <= max)) {
            currentAngle = imu.getAngle();
        }
        stopAndReset();

        updateTelemetry();
    }

    public void turnOrigin(double power) {
        double angle = Threader.get(UltroImu.class).getAngle();
        turn(power, angle);
    }

    public void intake(double leftPower, double rightPower) {
        map.getLeftIntake().setPower(leftPower);
        map.getRightIntake().setPower(rightPower);
    }

    public void conveyer(double power) {
        DcMotor motor = map.getConveyer();
        if(motor.getPower() == power) return;
        map.getConveyer().setPower(power);
    }

    public void autoArm(double posLeft, double posRight){
        if(posLeft != -100)
            map.getLeftAuto().setPosition(posLeft);
        if(posRight != -100)
            map.getRightAuto().setPosition(posRight);
    }

    public void prepareRight() {
        map.getRightAuto().setPosition(0.3);
        map.getRightFinger().setPosition(0.5);
    }
    public void closeRight() {
        map.getRightAuto().setPosition(0.6);
        map.getRightFinger().setPosition(0.0);
    }

    public void prepareLeft() {
        map.getLeftAuto().setPosition(0.9);
        map.getLeftFinger().setPosition(0.6);
    }
    public void closeLeft() {
        map.getLeftAuto().setPosition(0.6);
        map.getLeftFinger().setPosition(1.0);
    }

    @Override
    public void stop() {
        move(Direction.FORWARD, 0);
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public boolean motorsBusy() {
        DcMotor leftTop = map.getLeftTop();
        DcMotor rightTop = map.getRightTop();
        DcMotor leftBottom = map.getLeftBottom();
        DcMotor rightBottom = map.getRightBottom();

        return leftBottom.isBusy() && rightBottom.isBusy() && rightTop.isBusy() && leftTop.isBusy();
    }
    public DcMotor[] getMotors() {
        return motors;
    }

    public boolean isTest() {
        return test;
    }

    public void setTest(boolean test) {
        this.test = test;
    }

    public void addData(String header, Object value) {
        if(test) telemetry.addData(header, value);
    }
    public void updateTelemetry() {
        if(test) telemetry.update();
    }


    private DriveThread getDriveThread() {
        return Threader.get(DriveThread.class);
    }

    private void clearBulkCache() {
        if(test) return;
        map.clearBulkCache();

    }
}