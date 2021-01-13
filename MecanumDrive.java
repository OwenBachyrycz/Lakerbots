package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="mecanumDrive", group="Testing")

public class MecanumDrive extends OpMode {

    DcMotor frontLeft;
    DcMotor rearLeft;
    DcMotor frontRight;
    DcMotor rearRight;
    DcMotor launcherLeft;
    DcMotor launcherRight;
    Servo launcherServo;
    Servo armServo;
    Servo clawServo;

    double frontLeftPower;
    double rearLeftPower;
    double frontRightPower;
    double rearRightPower;
    double launcherLeftPower;
    double launcherRightPower;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");
        launcherLeft = hardwareMap.dcMotor.get("launcherLeft");
        launcherRight = hardwareMap.dcMotor.get("launcherRight");

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherServo = hardwareMap.servo.get("launcherArm");
        armServo = hardwareMap.servo.get("armServo");
        clawServo = hardwareMap.servo.get("clawServo");

        launcherServo.setPosition(0);
        armServo.setPosition(0);
        clawServo.setPosition(0);

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.5;
        double rx = gamepad1.right_stick_x;
        double launchPower = gamepad1.right_trigger;

        frontLeftPower = (y + x + rx);
        rearLeftPower = (y - x + rx);
        frontRightPower = (y - x - rx);
        rearRightPower = (y + x -rx);


        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);

        launcherLeft.setPower(launchPower);
        launcherRight.setPower(launchPower);

        if(gamepad1.dpad_down){
            launcherServo.setPosition(1);
        }

        if(gamepad1.dpad_up){
            launcherServo.setPosition(0);
        }

        while(gamepad1.a){
            if(armServo.getPosition() < 1.0){
                armServo.setPosition(armServo.getPosition() + .001);
            }
        }

        while(gamepad1.b){
            if(armServo.getPosition() > 0){
                armServo.setPosition(armServo.getPosition()- .001);
            }
        }

        while(gamepad1.x){
            clawServo.setPosition(1);
        }

        while(gamepad1.y){
            clawServo.setPosition(0);
        }
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);

        armServo.setPosition(0);
        clawServo.setPosition(0);
    }
}

