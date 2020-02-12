package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import net.jafama.FastMath;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.monitor.DeviceMap;
import org.firstinspires.ftc.teamcode.monitor.RobotData;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.firstinspires.ftc.teamcode.threading.Threader;
import org.firstinspires.ftc.teamcode.threading.control.UltroImu;

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
    private static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV/(WHEEL_DIAMETER_INCHES * Math.PI);

    public MecanumDriver() {
        this.map = DeviceMap.getInstance();
        this.motors = map.getDriveMotors();
        this.test = true;
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

    /**
     * Drive until a conditional is true
     * @param direction
     * @param power
     * @param conditional
     * @param gyroAssist
     */
    public void moveCond(Direction direction, double power, boolean conditional, boolean gyroAssist){
        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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

        if(direction == Direction.LEFT){
            map.getLeftTop().setPower(-power);
            map.getRightTop().setPower(power);
            map.getLeftBottom().setPower(power);
            map.getRightBottom().setPower(-power);
        } else if(direction == Direction.RIGHT){
            map.getLeftTop().setPower(power);
            map.getRightTop().setPower(-power);
            map.getLeftBottom().setPower(-power);
            map.getRightBottom().setPower(power);
        } else {
            move(direction, power);
        }
        //statement is an if because it is already placed inside of a while loop
//        if(!conditional) {
//            addData("moving to position", conditional);
//            updateTelemetry();
//            if ((gyroAssist && (direction == Direction.FORWARD)) || (gyroAssist && (direction == Direction.BACKWARD))){
//                double correctedPower = power * calculatePowerMultiplierLinear(0, angle, power);
//                addData("initial angle", initialAngle);
//                addData("angle", angle);
//                addData("power", power);
//                if (angle > initialAngle + 2) {
//                    addData("Increasing right side", correctedPower);
//                    gyroAssist(direction.getRightSide(), power + 0.1);
//                    gyroAssist(direction.getLeftSide(), correctedPower);
//                } else if(angle < initialAngle - 2) {
//                    addData("Increasing left side", correctedPower);
//                    gyroAssist(direction.getLeftSide(), power + 0.1);
//                    gyroAssist(direction.getRightSide(), correctedPower);
//                }else {
//                    addData("normal", "side");
//                    move(direction, power);
//                }
//                updateTelemetry();
//
//                UltroImu imu = Threader.get(UltroImu.class);
//                angle = imu.getAngle();
//            }
//        }
    }

    public void stopAndReset(){
        stop();
        addData("finished moving", 0.0);
        updateTelemetry();
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
        map.getLeftTop().setPower(direction.getLeftTop() * leftTop);
        map.getRightBottom().setPower(direction.getRightBottom() * rightBottom);
        map.getRightTop().setPower(direction.getRightTop() * rightTop);
        map.getLeftBottom().setPower(direction.getLeftBottom() * leftBottom);
    }

    @Override
    public void move(Direction direction, double power, double inches) {
        move(direction, power, inches, false);
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
        move(direction, power);

        while(linear.opModeIsActive() && motorsBusy(leftTopTarget, rightTopTarget, leftBottomTarget, rightBottomTarget)) {
            double[] powers = calculatePowerFromMotor(power, leftTopTarget, rightTopTarget, leftBottomTarget, rightBottomTarget);

            if (gyroAssist){
                gyroAssistor(powers, direction, initialAngle, angle, power);
                UltroImu imu = Threader.get(UltroImu.class);
                angle = imu.getAngle();
            }else move(direction, powers[0], powers[1], powers[2], powers[3]);
        }

        stop();

        for(DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void gyroAssistor(double[] powers, Direction direction, double initialAngle, double angle, double power) {
        double[] newPowers = new double[powers.length];
        for(double motorPower : powers) {
            double correctedPower = power * calculatePowerMultiplierLinear(0, angle, motorPower);
            if(angle > initialAngle + 2) {
                for(int id : direction.getRightSide()) {
                    newPowers[id] = motorPower + 0.1D;
                }
                for(int id : direction.getLeftSide()) {
                    newPowers[id] = correctedPower;
                }
            }else if(angle < initialAngle - 2) {
                for(int id : direction.getRightSide()) {
                    newPowers[id] = correctedPower;
                }
                for(int id : direction.getLeftSide()) {
                    newPowers[id] = motorPower + 0.1;
                }
            }else newPowers = powers;
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
        int[] target = new int[] { leftTopTarget, rightTopTarget, leftBottomTarget, rightBottomTarget };
        for(int i = 0; i < current.length; i++) {
            if(FastMath.abs(current[i]) < FastMath.abs(target[i])) return true;
        }
        return false;
    }

    public int[] getMotorCounts() {
        int leftTop = RobotData.leftTop;
        int rightTop = RobotData.rightTop;
        int leftBottom = RobotData.leftBottom;
        int rightBottom = RobotData.rightBottom;

        return new int[] { leftTop, rightTop, leftBottom, rightBottom};
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

    /**
     * Updated mecanum drive function this year (math is ? ?? ? )
     * @param left_stick_x gamepadleftX
     * @param left_stick_y gamepadleftY
     * @param right_stick_x gamepadrightX
     */
    public void moveTrigRed(double left_stick_x, double left_stick_y, double right_stick_x){
        // how much to amplify the power
        double r = Math.hypot(left_stick_y, left_stick_x);

        //calculates the angle of the joystick - 45 degrees
        double robotAngle = Math.atan2(left_stick_y, left_stick_x) - Math.PI / 4;

        // rotation
        double rightX = right_stick_x;

        // equation for each of the wheels
        final double LF = r * Math.cos(robotAngle) + rightX;
        final double RF = r * Math.sin(robotAngle) - rightX;
        final double LB = r * Math.sin(robotAngle) + rightX;
        final double RB = r * Math.cos(robotAngle) - rightX;

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
        double r = Math.hypot(left_stick_y, left_stick_x);

        //calculates the angle of the joystick - 45 degrees
        double robotAngle = Math.atan2(left_stick_y, left_stick_x) - ((5*Math.PI) / 4);

        // rotation
        double rightX = right_stick_x;

        // equation for each of the wheels
        final double LF = r * Math.cos(robotAngle) + rightX;
        final double RF = r * Math.sin(robotAngle) - rightX;
        final double LB = r * Math.sin(robotAngle) + rightX;
        final double RB = r * Math.cos(robotAngle) - rightX;

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
        stop();

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


    @Override
    public void stop() {
        for(DcMotor motor : map.getDriveMotors())
            motor.setPower(0);
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
}