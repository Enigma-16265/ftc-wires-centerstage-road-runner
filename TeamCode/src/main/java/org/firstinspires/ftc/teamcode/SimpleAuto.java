/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//do he auto and get a 2+2
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * ENIGMA Autonomous for with vision detection using EasyOpenCv and park
 */
//@Autonomous(name = "SIMPLE AUTO", group = "00-Autonomous", preselectTeleOp = "EvoWork")
public class SimpleAuto extends LinearOpMode {

    public static String TEAM_NAME = "ENIGMA"; //TODO: Enter team Name
    public static int TEAM_NUMBER = 16265; //TODO: Enter team Number

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime servoTimer = new ElapsedTime();

    private Servo rightLift;
    private Servo leftLift;
    private Servo shoulder;
    private Servo wrist;
    private Servo elbow;
    private Servo leftFinger;
    private Servo rightFinger;
    double LiftLeftOffset = .04;
    double LiftHeight;

    // Drive position for the arm
    private static final double WAIT_ONE_SEC = 1; // 1000 milliseconds
    private static final double WAIT_HALF_SEC = 0.5; // 500 milliseconds
    private static final double WAIT_QUARTER_SEC = 0.25; // 250 milliseconds
    private static final double WAIT_TENTH_SEC = 0.1; // 100 milliseconds
    private static final double WAIT_FIFTEENTH_SEC = 0.15; // 150 milliseconds
    private static final double WAIT_TWELFTH_SEC = 0.12; // 120 milliseconds

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    /*
    OpenCV / Color Detection
     */
    OpenCvCamera webcam1 = null;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
    public static double leftavgfinoutput = 0;
    public static double centeravgfinoutput = 0;
    public static double rightavgfinoutput = 0;

    private double servoposition = 0.0;
    private double servodelta = 0.02;
    private double servodelaytime = 0.03;

    // Servo functions
    private void moveServoGradually(Servo servo, double targetPosition) {
        double currentPosition = servo.getPosition();

        // Check if enough time has passed since the last update
        if (servoTimer.seconds() > servodelaytime) {
            // Determine the direction of movement
            double direction = targetPosition > currentPosition ? servodelta : -servodelta;

            // Calculate the new position
            servoposition = currentPosition + direction;
            servoposition = Range.clip(servoposition, 0, 1); // Ensure the position is within valid range

            // Update the servo position
            servo.setPosition(servoposition);

            // Reset the timer
            servoTimer.reset();
        }
    }

    private Servo setLiftPosition(double targetPosition) {
        // Ensure the target position is within the valid range
        targetPosition = Math.max(0.0, Math.min(targetPosition, 1.0));

        // Set the servo positions
        leftLift.setPosition(targetPosition);
        rightLift.setPosition(targetPosition);
        return null;
    }

    private void setLiftHeight(double inputLiftHeight) {
        if (inputLiftHeight < 0.42) {
            inputLiftHeight = 0.42;
        }
        if (inputLiftHeight > 1) {
            inputLiftHeight = 1;
        }
        LiftHeight = inputLiftHeight;
        leftLift.setPosition(LiftLeftOffset + LiftHeight);
        rightLift.setPosition(LiftHeight);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        elbow = hardwareMap.get(Servo.class, "elbow");
        wrist = hardwareMap.get(Servo.class, "wrist");
        leftFinger = hardwareMap.get(Servo.class, "lFinger");
        rightFinger = hardwareMap.get(Servo.class, "rFinger");

        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);


        //init pos
        setLiftPosition(EvoWork.LIFT_DRIVE);
        shoulder.setPosition(EvoWork.SHOULDER_DRIVE);
        wrist.setPosition(EvoWork.WRIST_INTAKE);
        elbow.setPosition(EvoWork.ELBOW_DRIVE);
        sleep(3000);
        leftFinger.setPosition(EvoWork.LEFT_FINGER_GRIP);
        rightFinger.setPosition(EvoWork.RIGHT_FINGER_GRIP);

        ArmController armController = new ArmController(shoulder, elbow, wrist, leftFinger, rightFinger);
        Thread armThread = new Thread(armController);
        armThread.start();

        // Vision OpenCV / Color Detection
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam1.setPipeline(new teamElementPipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.SENSOR_NATIVE);
            }
            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //Key inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);
        //waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);

            //Keep watching for the location of the team element.
            telemetry.addData("Vision identified Parking Location", identifiedSpikeMarkLocation);
            telemetry.addData("Cam Stream Preview (While INIT)", "3 dots, Camera Stream");
            telemetry.addData(">", "When ready Touch Play to start OpMode");
            //telemetry.addData("leftavfin", leftavgfinoutput);
            //telemetry.addData("centeravfin", centeravgfinoutput);
            //telemetry.addData("rightavfin", rightavgfinoutput);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            //Build trajectories based on the target location detected by vision
            runAutonoumousMode(armController);
        }

        armController.stop();
        armThread.join(); // Ensure the thread finishes execution
    }   // end runOpMode()
    public void runAutonoumousMode(ArmController armController) {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0); // after placing pixel
        Pose2d scorePose1 = new Pose2d(0,0,0);
        Pose2d scorePose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d intakeStack2 = new Pose2d(0,0,0);
        Pose2d intakeprep = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0,0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(8,0,0);

        //TODO: edit here
        switch (startPosition) {
            case BLUE_LEFT:
/*
                   | +y
                ________
                |>>>>
        <- -x   |>>>>    -> +x
                |>>>>
                --------
                   | -y
*/
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(16.27, 13.295, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(22.5, 35, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(23.25, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(27, 35.5,  Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(-33));
                        dropYellowPixelPose = new Pose2d(31, 35.5, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(55, 35, Math.toRadians(-90));
                intakeStack = new Pose2d(56, -55,Math.toRadians(-90));
                intakeprep = new Pose2d(56, -64.5,Math.toRadians(-90));
                intakeStack2 = new Pose2d(51.5, 35,Math.toRadians(-90));
                scorePose1 = new Pose2d(27, 31.5,Math.toRadians(-90));
                scorePose1a = new Pose2d(27, 31,Math.toRadians(-90));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                //used to be 2          ^  this goes for all of them
                parkPose = new Pose2d(8, 30, Math.toRadians(-90));
                break;

            case RED_RIGHT:
/*
                   | +y
                ________
                   <<<<|
        <- +x      <<<<|   -> -x
                   <<<<|
                --------
                   | -y
*/
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(21.75, 0, Math.toRadians(70));
                        dropYellowPixelPose = new Pose2d(34.25, -35, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(22.75, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(27, -35,  Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(-35));
                        dropYellowPixelPose = new Pose2d(19.75, -35, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(14, -13, Math.toRadians(45));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(8, -30, Math.toRadians(90));
                break;

            case BLUE_RIGHT:
/*
                   | +y
                ________
                |>>>>
        <- -x   |>>>>    -> +x
                |>>>>
                --------
                   | -y
*/
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(60));
                        dropYellowPixelPose = new Pose2d(21, 83, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(23.25, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(26, 83, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(-35));
                        dropYellowPixelPose = new Pose2d(31, 83, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(18, -21, Math.toRadians(-90));
                intakeStack = new Pose2d(53.25, -20,Math.toRadians(-90));
                intakeStack2 = new Pose2d(53.25, -15,Math.toRadians(-90));
                midwayPose2 = new Pose2d(52, 62, Math.toRadians(-90));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(50, 84, Math.toRadians(-90));
                break;

            case RED_LEFT:
/*
                   | +y
                ________
                   <<<<|
        <- +x      <<<<|   -> -x
                   <<<<|
                --------
                   | -y
*/
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(70));
                        dropYellowPixelPose = new Pose2d(34, -87.5, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(23.25, 0, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(29, -87.5, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(22, 0, Math.toRadians(-35));
                        dropYellowPixelPose = new Pose2d(23, -87.5, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(18, 21, Math.toRadians(90));
                intakeStack = new Pose2d(52, 19,Math.toRadians(90));
                midwayPose2 = new Pose2d(52, -62, Math.toRadians(90));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(50, -84, Math.toRadians(90));
                break;
        }

        /*
        // Position claw to drop the Purple Pixel on Spike Mark
        shoulder.setPosition(EvoWork.SHOULDER_DRIVE);
        wrist.setPosition(EvoWork.WRIST_INTAKE);
        for(int c = 0; c<44; c++) {
            moveServoGradually(elbow, EvoWork.ELBOW_INTAKE);
            safeWaitSeconds(WAIT_TWELFTH_SEC);
        }
        */


        armController.StartIntake();
        while (!armController.isActionCompleted()) {
            // Add a small delay to prevent tight looping
            sleep(10);
            // Optionally add telemetry here to debug
            telemetry.addData("Waiting for Action Completion", "");
            telemetry.update();
        }
        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());
        armController.DropPurple();
        armController.startDrive();
/*
        // prep to drive to the board
        shoulder.setPosition(EvoWork.SHOULDER_DRIVE);
        elbow.setPosition(EvoWork.ELBOW_DRIVE);
        leftFinger.setPosition(EvoWork.LEFT_FINGER_GRIP);
        rightFinger.setPosition(EvoWork.RIGHT_FINGER_GRIP);
        wrist.setPosition(EvoWork.WRIST_DRIVE);

        safeWaitSeconds(waitSecondsBeforeDrop);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        .build());


        //drop Pixel on Backdrop
        safeWaitSeconds(WAIT_QUARTER_SEC);
        shoulder.setPosition(EvoWork.SCORE_ZERO_SHOULDER);
        safeWaitSeconds(WAIT_QUARTER_SEC);
        wrist.setPosition(EvoWork.SCORE_ZERO_WRIST);
        safeWaitSeconds(WAIT_QUARTER_SEC);
        elbow.setPosition(EvoWork.SCORE_ZERO_ELBOW);
        safeWaitSeconds(WAIT_HALF_SEC);
        // drop yellow pixel
        leftFinger.setPosition(EvoWork.LEFT_FINGER_DROP);
        safeWaitSeconds(WAIT_QUARTER_SEC);
        // return to drive

        shoulder.setPosition(EvoWork.SHOULDER_DRIVE);
        elbow.setPosition(EvoWork.ELBOW_DRIVE);
        leftFinger.setPosition(EvoWork.LEFT_FINGER_GRIP);
        rightFinger.setPosition(EvoWork.RIGHT_FINGER_GRIP);
        wrist.setPosition(EvoWork.WRIST_DRIVE);

        if (startPosition == START_POSITION.BLUE_LEFT ||
                startPosition == START_POSITION.RED_RIGHT) {

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .build());
            //TODO : Code to intake pixel from stack
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());
            safeWaitSeconds(WAIT_QUARTER_SEC);
            rightFinger.setPosition(EvoWork.RIGHT_FINGER_INTAKE);
            safeWaitSeconds(WAIT_TENTH_SEC);
            shoulder.setPosition(EvoWork.SHOULDER_TOP_TWO);
            safeWaitSeconds(WAIT_TENTH_SEC);
            wrist.setPosition(EvoWork.WRIST_TOP_TWO);
            safeWaitSeconds(WAIT_HALF_SEC);
            elbow.setPosition(EvoWork.ELBOW_TOP_TWO);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(intakeprep.position, intakeprep.heading)
                            .build());
            safeWaitSeconds(WAIT_HALF_SEC);
            rightFinger.setPosition(EvoWork.RIGHT_FINGER_GRIP);

            // drive back to backboard and strafe to scoring position
            safeWaitSeconds(.2);
            shoulder.setPosition(EvoWork.SHOULDER_DRIVE);
            elbow.setPosition(EvoWork.ELBOW_DRIVE);
            wrist.setPosition(EvoWork.WRIST_DRIVE);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(intakeStack2.position, intakeStack2.heading)
                            .strafeToLinearHeading(scorePose1.position, scorePose1.heading)
                            .build());
            wrist.setPosition(EvoWork.SCORING_UPRIGHT_WRIST);
            safeWaitSeconds(WAIT_HALF_SEC);
            shoulder.setPosition(EvoWork.SCORING_UPRIGHT_SHOULDER);
            safeWaitSeconds(WAIT_HALF_SEC);
            elbow.setPosition(EvoWork.SCORING_UPRIGHT_ELBOW);
            safeWaitSeconds(WAIT_HALF_SEC);
            // drop first pixel
            rightFinger.setPosition(EvoWork.RIGHT_FINGER_DROP);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(scorePose1a.position, scorePose1a.heading)
                            .build());
            safeWaitSeconds(WAIT_TENTH_SEC);
            // drop second pixel

            // return to drive
            shoulder.setPosition(EvoWork.SHOULDER_DRIVE);
            elbow.setPosition(EvoWork.ELBOW_DRIVE);
            leftFinger.setPosition(EvoWork.LEFT_FINGER_GRIP);
            rightFinger.setPosition(EvoWork.RIGHT_FINGER_GRIP);
            wrist.setPosition(EvoWork.WRIST_DRIVE);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());
        }
        */
    }


    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing ENIGMA Autonomous Team# a",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
    class teamElementPipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat centerCrop;
        Mat rightCrop;
        double leftavgfin;
        double centeravgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            // spike frames for team element locations
            Rect leftRect = new Rect(1,380,125, 200);
            Rect centerRect = new Rect(500,300,200, 200);
            Rect rightRect = new Rect(1100,380,150, 200);

            input.copyTo(outPut);

            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, centerRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            centerCrop = YCbCr.submat(centerRect);
            rightCrop = YCbCr.submat(rightRect);

            // color refraction
            if (startPosition == START_POSITION.BLUE_LEFT || startPosition == START_POSITION.BLUE_RIGHT) {
                Core.extractChannel(leftCrop, leftCrop, 0);
                Core.extractChannel(centerCrop, centerCrop, 0);
                Core.extractChannel(rightCrop, rightCrop, 0);  // blue is 0, green is 1, red is 2
            } else if (startPosition == START_POSITION.RED_LEFT || startPosition == START_POSITION.RED_RIGHT) {
                Core.extractChannel(leftCrop, leftCrop, 2);
                Core.extractChannel(centerCrop, centerCrop, 2);
                Core.extractChannel(rightCrop, rightCrop, 2);  // blue is 0, green is 1, red is 2
            } else {
                Core.extractChannel(leftCrop, leftCrop, 0);
                Core.extractChannel(centerCrop, centerCrop, 0);
                Core.extractChannel(rightCrop, rightCrop, 0);  // blue is 0, green is 1, red is 2
            }

            Scalar leftavg = Core.mean(leftCrop);
            Scalar centeravg = Core.mean(centerCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            centeravgfin = centeravg.val[0];
            rightavgfin = rightavg.val[0];

            leftavgfinoutput = leftavgfin;
            centeravgfinoutput = centeravgfin;
            rightavgfinoutput = rightavgfin;

            if (leftavgfin < centeravgfin && leftavgfin < rightavgfin) {
                //telemetry.addLine("Left");
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
            } else if (centeravgfin < leftavgfin && centeravgfin < rightavgfin) {
                //telemetry.addLine("Center");
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
            } else if (rightavgfin < leftavgfin && rightavgfin < centeravgfin) {
                //telemetry.addLine("Right");
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
            } else {
                //telemetry.addLine("Failed to detect position");
                identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
            }
            return (outPut);
        }
    }

}   // end class