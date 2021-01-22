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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="CVAuto", group="test")
public class CVAuto extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor rearLeft;
    DcMotor frontRight;
    DcMotor rearRight;
    DcMotor launcherLeft;
    DcMotor launcherRight;
    Servo launcherArm;
    Servo pusherServo;
    Servo wobbleArm;
    Servo wobbleGrip;
    OpenCvCamera webcam;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {

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
        launcherArm = hardwareMap.servo.get("launcherArm");
        wobbleArm = hardwareMap.servo.get("wobbleArm");
        wobbleGrip = hardwareMap.servo.get("wobbleGrip");

        launcherArm.setPosition(0.085);
        wobbleGrip.setPosition(0);
        sleep(250);
        wobbleArm.setPosition(0.75);
        pusherServo.setPosition(0);

        //OpenCV and webcam initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        //Camera config
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        for(int i = 0; i < 10 && opModeIsActive(); i++)
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(20);
        }


        sleep(100);

        int ringAnalysis = pipeline.getAnalysis();

        char scenario = ' ';
        if(ringAnalysis >= 142){
            scenario = 'C';
        }
        else if(ringAnalysis >= 135){
            scenario = 'B';
        }
        else{
            scenario = 'A';
        }

        launcherLeft.setPower(1);
        launcherRight.setPower(1);

        moveDrivetrain(0, 0.5, -0.25, 250);

        sleep(500);
        pusherServo.setPosition(1);
        sleep(1000);
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
        pusherServo.setPosition(0);

        moveDrivetrain(0, 0.5, 0.17, 275);

        moveDrivetrain(-0.1, 0.6, 0.08, 1000);

        sleep(1000);

        if(scenario == 'A'){
            moveDrivetrain(0.4,0,0,500);
            //Release wobble goal
            sleep(400);
            wobbleArm.setPosition(0.2);
            sleep(1000);
            wobbleGrip.setPosition(1);
        }
        else if(scenario == 'B'){
            moveDrivetrain(-0.25,0.25,0,500);
            sleep(400);
            wobbleArm.setPosition(0.2);
            sleep((1000));
            wobbleGrip.setPosition(1);
            //Release wobble goal
        }
        else if(scenario == 'C'){
            moveDrivetrain(.4,1,0,750);
            sleep(400);
            wobbleArm.setPosition(0.2);
            sleep(1000);
            wobbleGrip.setPosition(1);
        }

    }

    /**
     * Sets drive train to move in x, y, and rx, for time seconds and then stops
     * @param x
     * @param y
     * @param rx
     * @param time
     */
    public void moveDrivetrain(double x, double y, double rx, int time){
        frontLeft.setPower(y + x + rx);
        rearLeft.setPower(y - x + rx);
        frontRight.setPower(y - x - rx);
        rearRight.setPower(y + x -rx);
        sleep(time);
        stopDrivetrain();
    }

    /**
     * Stops all drive train motors
     */
    public void stopDrivetrain(){
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {

        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(150, 300);

        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 100;

        final int FOUR_RING_THRESHOLD = 145;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        private volatile EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.ONE;
            } else {
                position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }
}
