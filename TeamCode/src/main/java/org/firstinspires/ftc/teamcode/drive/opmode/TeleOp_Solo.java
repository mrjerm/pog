package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.Constants.DR4B_Cap;
import static org.firstinspires.ftc.teamcode.drive.Constants.DR4B_High;
import static org.firstinspires.ftc.teamcode.drive.Constants.DR4B_Low;
import static org.firstinspires.ftc.teamcode.drive.Constants.DR4B_Mid;
import static org.firstinspires.ftc.teamcode.drive.Constants.DR4B_Rest;
import static org.firstinspires.ftc.teamcode.drive.Constants.clawClosePos;
import static org.firstinspires.ftc.teamcode.drive.Constants.clawOpenPos;
import static org.firstinspires.ftc.teamcode.drive.Constants.clawRestPos;
import static org.firstinspires.ftc.teamcode.drive.Constants.horizontalSlideL3;
import static org.firstinspires.ftc.teamcode.drive.Constants.horizontalSpeedLimit;
import static org.firstinspires.ftc.teamcode.drive.Constants.odometerDownPos;
import static org.firstinspires.ftc.teamcode.drive.Constants.odometerUpPos;
import static org.firstinspires.ftc.teamcode.drive.Constants.salamiSpeed;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp_Solo extends OpMode {

    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx horizontalSlide;
    public DcMotorEx intake;

    public Servo DR4BServo;

    public CRServo duckSpinnerLeft;
    public CRServo duckSpinnerRight;

    public Servo clawServo;
    public Servo odometerYL;
    public Servo odometerYR;
    public Servo odometerX;

    public ColorSensor colorSensor;

    //init variables
    boolean slomo = true;
    double horizontalSlidePower = 0;

    boolean changeLiftModePrevious = false;
    boolean moveUpPrevious = false;
    boolean moveDownPrevious = false;
    public double time = 0;
    public boolean freightDetected = false;
    public boolean readyToLift = false;
    public boolean timeResetted = false;
    public boolean freightInGrabber = false;
    public double timer = 0;

    public enum LiftMode {
        MACRO,
        MICRO;

        public LiftMode flip() {
            switch (this) {
                case MACRO:
                    return MICRO;
                case MICRO:
                    return MACRO;
                default:
                    return MACRO;
            }
        }
    }

    LiftMode liftMode = LiftMode.MACRO;

    public enum LiftState {
        REST,
        LOW,
        MID,
        HIGH,
        CAP;

        public LiftState next() {
            switch (this) {
                case REST:
                    return LOW;
                case LOW:
                    return MID;
                case MID:
                    return HIGH;
                case HIGH:
                case CAP:
                    return CAP;
                default:
                    return REST;
            }
        }

        public LiftState previous() {
            switch (this) {
                case CAP:
                    return HIGH;
                case HIGH:
                    return MID;
                case MID:
                    return LOW;
                case REST:
                case LOW:
                default:
                    return REST;
            }
        }
    }

    LiftState liftState = LiftState.REST;

    public enum ClawState {
        CLAW_OPEN,
        CLAW_ClOSE,
        CLAW_REST
    }

    ;

    ClawState clawState = ClawState.CLAW_REST;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "Motor FL");
        backLeft = hardwareMap.get(DcMotorEx.class, "Motor BL");
        frontRight = hardwareMap.get(DcMotorEx.class, "Motor FR");
        backRight = hardwareMap.get(DcMotorEx.class, "Motor BR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        horizontalSlide = hardwareMap.get(DcMotorEx.class, "Motor Horizontal Slide");
        horizontalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intake = hardwareMap.get(DcMotorEx.class, "Motor Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        DR4BServo = hardwareMap.get(Servo.class, "Servo DR4B");
        DR4BServo.setDirection(Servo.Direction.FORWARD);

        duckSpinnerLeft = hardwareMap.get(CRServo.class, "Servo Duck Spinner Left");
        duckSpinnerRight = hardwareMap.get(CRServo.class, "Servo Duck Spinner Right");
        duckSpinnerLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        duckSpinnerRight.setDirection(DcMotorSimple.Direction.REVERSE);
        /*TODO: Reverse one of these. */

        clawServo = hardwareMap.get(Servo.class, "Servo Claw");
        clawServo.setDirection(Servo.Direction.FORWARD);

        odometerYL = hardwareMap.get(Servo.class, "Servo Odometer YL");
        odometerYL.setDirection(Servo.Direction.REVERSE);

        odometerYR = hardwareMap.get(Servo.class, "Servo Odometer YR");
        odometerYR.setDirection(Servo.Direction.FORWARD);

        odometerX = hardwareMap.get(Servo.class, "Servo Odometer X");
        odometerX.setDirection(Servo.Direction.REVERSE);

        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");

        telemetry.addData("Status", "Ready!");
        telemetry.update();

    }


    @Override
    public void loop() {
        liftOdometers();
        drive(slomo);
        intek(gamepad1.right_bumper, gamepad1.left_bumper);
        ducky(gamepad1.dpad_left, gamepad1.dpad_right);
        grabby(gamepad1.left_trigger, gamepad1.right_trigger);
        setSlomo(gamepad1.a, gamepad1.y);
        isFreightInGrabber();
        extend(isFreightInGrabber());
        closey();
    }

    public boolean isFreightInGrabber (){
        if (colorSensor.red() > 400){
            freightDetected = true;
        }
        if (colorSensor.red() < 400){
            freightDetected = false;
        }
        return freightDetected;
    }

    public void extend (boolean freightInGrabber) {
        if (freightInGrabber){
            intake.setPower(-0.8);
            timer = getRuntime();
            while (getRuntime() - timer <= 0.3){
                drive(slomo);
                if (gamepad1.x){
                    break;
                }
            }
            liftState = LiftState.HIGH;
            clawState = ClawState.CLAW_ClOSE;
            timer = getRuntime();
            while (getRuntime() - timer <= 0.7){
                drive(slomo);
                if (gamepad1.x){
                    break;
                }
            }
            setHorizontalSlide(horizontalSlideL3, 1);
        }
    }

    public void retract (boolean keybind) {
        if (keybind) {
            clawState = ClawState.CLAW_REST;
            setHorizontalSlide(0, 0.8);
            timer = getRuntime();
            while (getRuntime() - timer <= 0.8){
                drive(slomo);
            }
        }
    }
    public void setHorizontalSlide(int position, double power){
        horizontalSlide.setTargetPosition(position);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlide.setPower(power);
    }

    public void closey(){
        if (liftState == LiftState.LOW || liftState == LiftState.REST){
            clawState = ClawState.CLAW_REST;
        }
    }


    public void liftOdometers(){
            odometerYL.setPosition(odometerUpPos);
            odometerYR.setPosition(odometerUpPos);
            odometerX.setPosition(odometerUpPos);
    }

    public void lowerOdometers(boolean req){
        if (req){
            odometerYL.setPosition(odometerDownPos);
            odometerYR.setPosition(odometerDownPos);
            odometerX.setPosition(odometerDownPos);
        }
    }

    public void midPiece(boolean keybind){
        if (keybind){
            clawState = ClawState.CLAW_ClOSE;
            liftState = LiftState.MID;
        }
    }

    public void ducky(boolean left, boolean right){
        duckSpinnerLeft.setPower(left ? 1 : 0);
        duckSpinnerRight.setPower(right ? -1 : 0);
    }

    public void switchLiftMode(boolean keybind){
        boolean changeliftModeCurrent = keybind;
        if (changeliftModeCurrent && !changeLiftModePrevious) {
            liftMode = liftMode.flip();
        }
        changeLiftModePrevious = changeliftModeCurrent;
    }

    public void lifty(boolean up, boolean down){
        if (liftMode == LiftMode.MACRO) {
            boolean moveUpCurrent = up;
            if (moveUpCurrent && !moveUpPrevious) {
                liftState = liftState.next();
            }
            moveUpPrevious = moveUpCurrent;

            boolean moveDownCurrent = down;
            if (moveDownCurrent && !moveDownPrevious) {
                liftState = liftState.previous();
            }
            moveDownPrevious = moveDownCurrent;

            switch (liftState) {
                case REST:
                    setDR4BServo(DR4B_Rest);
                    break;
                case LOW:
                    setDR4BServo(DR4B_Low);
                    break;
                case MID:
                    setDR4BServo(DR4B_Mid);
                    break;
                case HIGH:
                    setDR4BServo(DR4B_High);
                    break;
                case CAP:
                    setDR4BServo(DR4B_Cap);
                    break;
                default:
                    telemetry.addData("DR4B Status", "messed up lmfao 😀");
                    telemetry.update();
            }

            telemetry.addData("DR4B State", liftState);
            telemetry.update();
        }
        if (liftMode == LiftMode.MICRO){
            boolean moveUpCurrent = up;
            if (moveUpCurrent && !moveUpPrevious) {
                setDR4BServo(DR4BServo.getPosition() + 0.05);
            }
            moveUpPrevious = moveUpCurrent;

            boolean moveDownCurrent = down;
            if (moveDownCurrent && !moveDownPrevious) {
                setDR4BServo(DR4BServo.getPosition() - 0.05);
            }
            moveDownPrevious = moveDownCurrent;
        }
        telemetry.addData("DR4B Position", DR4BServo.getPosition());
        telemetry.update();
    }

    public void setDR4BServo(double position){
        DR4BServo.setPosition(position);
    }

    public void slidey(boolean forward, boolean backward){
        if (forward) {
            horizontalSlide.setPower(horizontalSpeedLimit * 0.7);
        }
        if (backward) {
            horizontalSlide.setPower(-1 * horizontalSpeedLimit * 0.7);
        }
    }

    public void intek(boolean forward, boolean reverse){
        if (forward){
            intake.setPower(0.8);
        }
        if (reverse){
            intake.setPower(-0.8);
        }
        if (!forward && !reverse){
            intake.setPower(0);
        }
    }

    public void setClawServo(double position) {
        clawServo.setPosition(position);
    }

    public void grabby(double open, double close) {
        if (open != 0) {
            clawState = ClawState.CLAW_OPEN;
        }
        if (close != 0) {
            clawState = ClawState.CLAW_ClOSE;
        }

        switch (clawState){
            case CLAW_OPEN:
                setClawServo(clawOpenPos);
                break;
            case CLAW_ClOSE:
                setClawServo(clawClosePos);
                break;
            case CLAW_REST:
                setClawServo(clawRestPos);
                break;
            default:
                telemetry.addData("Claw Status", "😹👎");
                telemetry.update();

        }
    }

    public void drive(boolean slomo){
        /*TODO: add fast and slow modes. Fancier math for right stick x
         */
        float x1 = slomo ? (float) (-gamepad1.left_stick_x * salamiSpeed) : -gamepad1.left_stick_x;
        float y1 = slomo ? (float) (-gamepad1.left_stick_y * salamiSpeed) : -gamepad1.left_stick_y;
        float x2 = slomo ? (float) ((float) Math.pow(gamepad1.right_stick_x, 1) * salamiSpeed * 0.75) : (float) Math.pow(gamepad1.right_stick_x, 3);

        double fL = -x1 + y1 + x2;
        double bL = x1 + y1 + x2;
        double fR = x1 + y1 - x2;
        double bR = -x1 + y1 - x2;

        frontLeft.setPower(fL);
        backLeft.setPower(bL);
        frontRight.setPower(fR);
        backRight.setPower(bR);
    }

    public boolean setSlomo(boolean on, boolean off){
        slomo = on || slomo;
        slomo = !off && slomo;
        return slomo;
    }
}