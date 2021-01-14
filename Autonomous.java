package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="mecanumAuto", group="test")
public class Autonomous extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor rearLeft;
    DcMotor frontRight;
    DcMotor rearRight;
    DcMotor launcherLeft;
    DcMotor launcherRight;
    Servo launcherServo;
    Servo armServo;
    Servo clawServo;
    Servo pusherServo;
    OpenCvCamera webcam;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");
        launcherLeft = hardwareMap.dcMotor.get("launcherLeft");
        launcherRight = hardwareMap.dcMotor.get("launcherRight");

        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        pusherServo = hardwareMap.servo.get("pusherServo");
        launcherServo = hardwareMap.servo.get("launcherArm");
        armServo = hardwareMap.servo.get("armServo");
        clawServo = hardwareMap.servo.get("clawServo");

        launcherServo.setPosition(0);
        armServo.setPosition(0);
        clawServo.setPosition(0);
        pusherServo.setPosition(0);


        /**
         * Sets up the webcam for OpenCV
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);


        /**
         * Starts the webcam and sets resolution and orientation
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        MecanumMovement move = new MecanumMovement(0,0,0); // Initialize mecanum drivetrain calculator

        int ringAnalysis = 12956;


        telemetry.addData("Analysis", ringAnalysis);

        launcherLeft.setPower(1);
        launcherRight.setPower(1);
        move.setMovement(0.5, 0, -0.15);
        setDrivetrain(move);
        sleep(250);
        stopDrivetrain();

        launcherServo.setPosition(0.075);
        sleep(500);
        pusherServo.setPosition(1);
        sleep(1000);
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
        pusherServo.setPosition(0);

        move.setMovement(0.5,0, 0.1);
        setDrivetrain(move);
        sleep(250);
        move.setMovement(1,0,0);
        setDrivetrain(move);
        sleep(650);
        stopDrivetrain();

    }

    /**
     * Sets drivetrain motor powers for direction "move" is set to
     * @param move
     */
    public void setDrivetrain(MecanumMovement move){
        frontLeft.setPower(move.getFrontLeft());
        rearLeft.setPower(move.getRearLeft());
        frontRight.setPower(move.getFrontRight());
        rearRight.setPower(move.getRearRight());
    }

    /**
     * Stops all drivetrain motors
     */
    public void stopDrivetrain(){
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }
}
