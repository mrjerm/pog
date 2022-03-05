package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
@Disabled

public class ColorTest extends OpMode {

    public ColorSensor colorSensor;
    public DcMotorEx intake;
    public double time = 0;
    public boolean freightDetected = false;
    public boolean readyToLift = false;
    public boolean timeResetted = false;
    public boolean freightInGrabber = false;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "Motor Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
    }

    @Override
    public void loop() {
        if (colorSensor.red() > 400){
            freightDetected = true;
        }
        else if (colorSensor.red() < 400){
            freightDetected = false;
        }
        if (freightDetected){
            intake.setPower(-0.8);
        }
        else if (!freightDetected && gamepad2.right_bumper){
            intake.setPower(0.8);
        }
        else if (!freightDetected && !gamepad2.right_bumper){
            intake.setPower(0);
        }
        telemetry.addData("red value", colorSensor.red());
        telemetry.addData("freight detected?", freightDetected);
        telemetry.update();
    }
}
