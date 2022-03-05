/**TODO: Make DR4B move up a little bit after dropping off at Level 2
 * TODO: Lower DR4B sooner after dropping off 2nd Freight
 * TODO: Move 2 inches less into the warehouse when collecting Freight
 *
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.Constants.DR4B_High;
import static org.firstinspires.ftc.teamcode.drive.Constants.DR4B_Low;
import static org.firstinspires.ftc.teamcode.drive.Constants.DR4B_Mid;
import static org.firstinspires.ftc.teamcode.drive.Constants.DR4B_Rest;
import static org.firstinspires.ftc.teamcode.drive.Constants.clawClosePos;
import static org.firstinspires.ftc.teamcode.drive.Constants.clawOpenPos;
import static org.firstinspires.ftc.teamcode.drive.Constants.clawRestPos;
import static org.firstinspires.ftc.teamcode.drive.Constants.horizontalSlideClear;
import static org.firstinspires.ftc.teamcode.drive.Constants.horizontalSlideL1;
import static org.firstinspires.ftc.teamcode.drive.Constants.horizontalSlideL2;
import static org.firstinspires.ftc.teamcode.drive.Constants.horizontalSlideL3;
import static org.firstinspires.ftc.teamcode.drive.Constants.odometerDownPos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous
public class Red_Warehouse extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "//sdcard//FIRST//tflitemodels//TSE.tflite";
    private static final String[] LABELS = {
            "TSE"
    };

    private static final String VUFORIA_KEY =
            "AaUAmHT/////AAABmQL+ceRLoET2hku+clLNd6BvmbZSle08MZLdzABXX4GKMlcRRBQrwbDvB3q+r5GR3htBm+3qzUNxoEaXXIf9OsDFNSbI8LvcFqQBjImS5C0lrylUrPWb/XxIhtUvxDgC9cpgrXPcdxJmlVJ3IIjVZ9vGODHbC8IekNOlNcF9Wpnnbv0YcJBqUOzdhA5YlY3Q5cE59qt5e1CZOnGoWeFo60S1L2zxtlRGgP4eTwD3pMSl9vZVQrq5WHvUuJTaTnJPqbkRWtdvmgm9b80hPeeY72DUJtDMqGSB5yrXAMhBLk6CuTxkgLQJ9YulzV+0DiaSx2RNK7NXQncFKcWfvPcmazGY/GVixkERJV8ONjqOfQq6";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private int dropLevel = 0;

    public DcMotorEx horizontalSlide;
    public DcMotorEx intake;

    public Servo DR4BServo;

    public CRServo duckSpinnerLeft;
    public CRServo duckSpinnerRight;

    public Servo clawServo;
    public Servo odometerYL;
    public Servo odometerYR;
    public Servo odometerX;


    @Override
    public void runOpMode() throws InterruptedException {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1, 16.0 / 9.0);
        }

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(7.8, -61.625, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        horizontalSlide = hardwareMap.get(DcMotorEx.class, "Motor Horizontal Slide");
        horizontalSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "Motor Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        DR4BServo = hardwareMap.get(Servo.class, "Servo DR4B");
        DR4BServo.setDirection(Servo.Direction.FORWARD);

        duckSpinnerLeft = hardwareMap.get(CRServo.class, "Servo Duck Spinner Left");
        duckSpinnerRight = hardwareMap.get(CRServo.class, "Servo Duck Spinner Right");
        duckSpinnerLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        duckSpinnerRight.setDirection(DcMotorSimple.Direction.REVERSE);

        clawServo = hardwareMap.get(Servo.class, "Servo Claw");
        clawServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setPosition(clawClosePos);

        odometerYL = hardwareMap.get(Servo.class, "Servo Odometer YL");
        odometerYL.setDirection(Servo.Direction.REVERSE);

        odometerYR = hardwareMap.get(Servo.class, "Servo Odometer YR");
        odometerYR.setDirection(Servo.Direction.FORWARD);

        odometerX = hardwareMap.get(Servo.class, "Servo Odometer X");
        odometerX.setDirection(Servo.Direction.REVERSE);

        odometerYL.setPosition(odometerDownPos);
        odometerYR.setPosition(odometerDownPos);
        odometerX.setPosition(odometerDownPos);

        int duckSpinTime = 3000;

        TrajectorySequence driveTraj = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    if (dropLevel == 1 || dropLevel == 2) {
                        setDR4BServo(DR4B_Mid);
                    }
                    if (dropLevel == 3){
                        setDR4BServo(DR4B_High);
                    }
                })
                .addTemporalMarker(0.5, () -> {
                    setHorizontalSlide(horizontalSlideClear, 1);
                })
                .addTemporalMarker(0.9, () -> {
                    if (dropLevel == 1) {
                        setDR4BServo(DR4B_Low);
                    }
                })
                .addTemporalMarker(1.1, () -> {
                    if (dropLevel == 1){
                        setHorizontalSlide(horizontalSlideL1, 1);
                    }
                    if (dropLevel == 2){
                        setHorizontalSlide(horizontalSlideL2, 1);
                    }
                    if (dropLevel == 3){
                        setHorizontalSlide(horizontalSlideL3, 0.8);
                    }
                })
                .lineToLinearHeading(new Pose2d(-1.52, -26, Math.toRadians(313.9809))) //go to shipping hub
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    clawServo.setPosition(clawOpenPos);
                })
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(driveTraj.end())
                .waitSeconds(0.7)

                .addDisplacementMarker(() -> {
                    if (dropLevel == 1){
                        setHorizontalSlide(horizontalSlideClear, 1);
                    }
                    if (dropLevel == 2 || dropLevel == 3){
                        setHorizontalSlide(0, 1);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    clawServo.setPosition(clawRestPos);
                    setDR4BServo(DR4B_Mid);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    setHorizontalSlide(0, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    setDR4BServo(DR4B_Rest);
                })
                .lineToLinearHeading(new Pose2d(4.2, -47.8, 0)) //go to barrier

                .lineToLinearHeading(new Pose2d(4.2, -56.6, Math.toRadians(3))) //align with barrier
                .addDisplacementMarker(() -> {
                    intake.setPower(0.8);
                })
                .lineToConstantHeading(new Vector2d(43, -55.6)) //go into warehouse
                .addDisplacementMarker(() -> {
                    clawServo.setPosition(clawClosePos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    setDR4BServo(DR4B_High);
                    intake.setPower(-0.8);

                })
                .lineToConstantHeading(new Vector2d(2.8, -55.6)) //go out of warehouse
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    setHorizontalSlide(horizontalSlideL3, 0.8);
                })
                .lineToLinearHeading(new Pose2d(-1.52, -26, Math.toRadians(313.9809))) //go to shipping hub
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    clawServo.setPosition(clawOpenPos);
                })
                .build();
        TrajectorySequence traj2part2 = drive.trajectorySequenceBuilder(traj2.end())
                .waitSeconds(0.7)

                .addDisplacementMarker(() -> {
                    if (dropLevel == 1){
                        setHorizontalSlide(horizontalSlideClear, 1);
                    }
                    if (dropLevel == 2 || dropLevel == 3){
                        setHorizontalSlide(0, 1);
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    clawServo.setPosition(clawRestPos);
                    setDR4BServo(DR4B_Mid);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    setHorizontalSlide(0, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    setDR4BServo(DR4B_Rest);
                })
                .lineToLinearHeading(new Pose2d(4.2, -47.8, 0)) //go to barrier

                .lineToLinearHeading(new Pose2d(4.2, -56.6, Math.toRadians(3))) //align with barrier
                .addDisplacementMarker(() -> {
                    intake.setPower(0.8);
                })
                .lineToConstantHeading(new Vector2d(47, -55.6)) //go into warehouse
                .addDisplacementMarker(() -> {
                    clawServo.setPosition(clawClosePos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    intake.setPower(-0.8);
                    setDR4BServo(DR4B_High);

                })
                .lineToConstantHeading(new Vector2d(2.8, -55)) //go out of warehouse
                .addDisplacementMarker(() -> {
                    intake.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    setHorizontalSlide(horizontalSlideL3, 0.8);
                })
                .lineToLinearHeading(new Pose2d(-1.52, -26, Math.toRadians(313.9809))) //go to shipping hub
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    clawServo.setPosition(clawOpenPos);
                })
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2part2.end())
                .waitSeconds(0.7)
                .addDisplacementMarker(() -> {
                    setHorizontalSlide(0, 1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    clawServo.setPosition(clawRestPos);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    setDR4BServo(DR4B_Rest);
                })
                .lineToLinearHeading(new Pose2d(6.3, -55, 0)) //go to barrier

                .lineToLinearHeading(new Pose2d(35, -54.2, 0)) //go into warehouse

                .lineToLinearHeading(new Pose2d(35, -31, 0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 0.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .lineToLinearHeading(new Pose2d(50, -29, 0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 1.3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        while (!isStarted() && !isStopRequested()){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.isEmpty()) {
                        dropLevel = 3;
                        telemetry.addData("Drop Level", dropLevel);
                        telemetry.update();
                    } else {
                        for (int scanner = 0; scanner < updatedRecognitions.size(); scanner++) {
                            if (updatedRecognitions.get(scanner).getLabel() == LABELS[0]) {
                                if (updatedRecognitions.get(scanner).getLeft() <= 320) {
                                    dropLevel = 1;
                                }
                                else if (updatedRecognitions.get(scanner).getLeft() >= 320) {
                                    dropLevel = 2;
                                }
                                telemetry.addData("Left", updatedRecognitions.get(scanner).getLeft());
                                telemetry.addData("Drop Level", dropLevel);
                                telemetry.update();
                            }
                        }
                    }
                }
            }

        }
        if (opModeIsActive()){

            drive.followTrajectorySequence(driveTraj);
            drive.followTrajectorySequence(traj2);
            drive.followTrajectorySequence(traj2part2);
            drive.followTrajectorySequence(traj3);
        }
    }

    private void setHorizontalSlide(int position, double power){
        horizontalSlide.setTargetPosition(position);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlide.setPower(power);
    }

    private void setDR4BServo(double position){
        DR4BServo.setPosition(position);
    }

    /**
     * Initialize the Vuforia localization engine.
     */

    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}
