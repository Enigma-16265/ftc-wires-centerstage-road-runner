package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ArmController implements Runnable {
    private ElapsedTime servoTimer = new ElapsedTime();
    private volatile boolean opModeActive = true;
    private volatile boolean running = true;

    // Servos
    private Servo shoulder, elbow, wrist, leftFinger, rightFinger;

    private double servoposition = 0.0;
    private double servodelta = 0.02;
    private double servodelaytime = 0.03;

    public ArmController(Servo shoulder, Servo elbow, Servo wrist, Servo leftFinger, Servo rightFinger) {
        this.shoulder = shoulder;
        this.elbow = elbow;
        this.wrist = wrist;
        this.leftFinger = leftFinger;
        this.rightFinger = rightFinger;
    }


    public void setOpModeActive(boolean active) {
        this.opModeActive = active;
    }

    // State machine for Claw Fingers
    private enum ClawState { IDLE, MOVE_RIGHT_FINGER, MOVE_LEFT_FINGER, MOVE_BOTH_FINGERS, START_MOVING, COMPLETED }
    private volatile ClawState currentClawState = ClawState.IDLE;
    private ClawPosition activeClawPosition;

    private boolean inClawPos = false;

    static class ClawPosition {
        double leftFingerPosition;
        double rightFingerPosition;
        double accelerationMax;
        double velocityMax;

        public ClawPosition(double leftFingerPos, double rightFingerPos,
                             double accMax, double velMax) {
            this.leftFingerPosition = leftFingerPos;
            this.rightFingerPosition = rightFingerPos;
            this.accelerationMax = accMax;
            this.velocityMax = velMax;
        }
    }

    private void handleClawSequence(ArmController.ClawPosition clawPos) {
        switch (currentClawState) {
            case START_MOVING:
            case MOVE_LEFT_FINGER:
                // Move the left finger position
                leftFinger.setPosition(clawPos.leftFingerPosition);
                currentClawState = ClawState.COMPLETED;
                break;
            case MOVE_RIGHT_FINGER:
                // Move the right finger position
                rightFinger.setPosition(clawPos.rightFingerPosition);
                currentClawState = ClawState.COMPLETED;
                break;
            case MOVE_BOTH_FINGERS:
                // Move the both finger positions
                leftFinger.setPosition(clawPos.leftFingerPosition);
                rightFinger.setPosition(clawPos.rightFingerPosition);
                currentClawState = ClawState.COMPLETED;
                break;
            case COMPLETED:

                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentClawState == ClawState.COMPLETED) {
            inClawPos = false;
            currentClawState = ClawState.IDLE;
            activeClawPosition = null; // Reset the active position
        }
    }

    public void DropPurple() {
        activeClawPosition = new ClawPosition(EvoWork.LEFT_FINGER_GRIP, EvoWork.RIGHT_FINGER_DROP, EvoWork.HIGH_ACC, EvoWork.HIGH_VEL);
        currentClawState = ClawState.MOVE_BOTH_FINGERS;
    }

    // State machine for Scoring
    private enum ScoreState { IDLE, MOVING_SHOULDER, MOVING_WRIST, MOVING_ELBOW, START_MOVING, COMPLETED }
    private volatile ScoreState currentScoreState = ScoreState.IDLE;
    private ScorePosition activeScorePosition;

    private boolean inScorePos = false;

    static class ScorePosition {
        double shoulderPosition;
        double elbowPosition;
        double wristPosition;
        double leftFingerPosition;
        double rightFingerPosition;
        double accelerationMax;
        double velocityMax;

        public ScorePosition(double shoulderPos, double elbowPos, double wristPos,
                             double leftFingerPos, double rightFingerPos,
                             double accMax, double velMax) {
            this.shoulderPosition = shoulderPos;
            this.elbowPosition = elbowPos;
            this.wristPosition = wristPos;
            this.leftFingerPosition = leftFingerPos;
            this.rightFingerPosition = rightFingerPos;
            this.accelerationMax = accMax;
            this.velocityMax = velMax;
        }
    }

    private void handleScoreSequence(ArmController.ScorePosition scorePos) {
        switch (currentScoreState) {
            case START_MOVING:
            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                moveServoWithTrapezoidalVelocity(shoulder, scorePos.shoulderPosition, scorePos.accelerationMax, scorePos.velocityMax);
                if (isServoAtPosition(shoulder, scorePos.shoulderPosition, EvoWork.SERVO_TOLERANCE)) {
                    currentScoreState = ScoreState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to intake position so it don't slap the floor
                leftFinger.setPosition(EvoWork.LEFT_FINGER_GRIP);
                rightFinger.setPosition(EvoWork.RIGHT_FINGER_GRIP);
                moveServoWithTrapezoidalVelocity(elbow, scorePos.elbowPosition, scorePos.accelerationMax, scorePos.velocityMax);
                if (isServoAtPosition(elbow, scorePos.elbowPosition, EvoWork.SERVO_TOLERANCE)) {
                    currentScoreState = ScoreState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                moveServoWithTrapezoidalVelocity(wrist, scorePos.wristPosition, scorePos.accelerationMax, scorePos.velocityMax);
                if (isServoAtPosition(wrist, scorePos.wristPosition, EvoWork.SERVO_TOLERANCE)) {
                    currentScoreState = ScoreState.COMPLETED;
                }
                break;
            case COMPLETED:

                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentScoreState == ScoreState.COMPLETED) {
            inIntakePos = false;
            currentScoreState = ScoreState.IDLE;
            activeScorePosition = null; // Reset the active position
        }
    }

    public void ScoreYellow() {
        activeScorePosition = new ScorePosition(EvoWork.SHOULDER_DRIVE, EvoWork.ELBOW_DRIVE, EvoWork.WRIST_DRIVE, EvoWork.LEFT_FINGER_GRIP, EvoWork.RIGHT_FINGER_GRIP, EvoWork.HIGH_ACC, EvoWork.HIGH_VEL);
        currentScoreState = ScoreState.MOVING_SHOULDER;
    }

    // State machine for Drive
    private enum DriveState { IDLE, MOVING_SHOULDER, MOVING_WRIST, MOVING_ELBOW, MOVING_CLAWS, START_MOVING, COMPLETED }
    private volatile DriveState currentDriveState = DriveState.IDLE;
    private DrivePosition activeDrivePosition;

    private boolean inDrivePos = false;

    static class DrivePosition {
        double shoulderPosition;
        double elbowPosition;
        double wristPosition;
        double leftFingerPosition;
        double rightFingerPosition;
        double accelerationMax;
        double velocityMax;

        public DrivePosition(double shoulderPos, double elbowPos, double wristPos,
                              double leftFingerPos, double rightFingerPos,
                              double accMax, double velMax) {
            this.shoulderPosition = shoulderPos;
            this.elbowPosition = elbowPos;
            this.wristPosition = wristPos;
            this.leftFingerPosition = leftFingerPos;
            this.rightFingerPosition = rightFingerPos;
            this.accelerationMax = accMax;
            this.velocityMax = velMax;
        }
    }

    private void handleDriveSequence(ArmController.DrivePosition drivePos) {
        switch (currentDriveState) {
            case START_MOVING:
            case MOVING_ELBOW:
                // Move the elbow to intake position so it don't slap the floor
                leftFinger.setPosition(EvoWork.LEFT_FINGER_GRIP);
                rightFinger.setPosition(EvoWork.RIGHT_FINGER_GRIP);
                moveServoWithTrapezoidalVelocity(elbow, drivePos.elbowPosition, drivePos.accelerationMax, drivePos.velocityMax);
                if (isServoAtPosition(elbow, drivePos.elbowPosition, EvoWork.SERVO_TOLERANCE)) {
                    currentDriveState = DriveState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                moveServoWithTrapezoidalVelocity(wrist, drivePos.wristPosition, drivePos.accelerationMax, drivePos.velocityMax);
                if (isServoAtPosition(wrist, drivePos.wristPosition, EvoWork.SERVO_TOLERANCE)) {
                    currentDriveState = DriveState.MOVING_SHOULDER;
                }
                break;
            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                moveServoWithTrapezoidalVelocity(shoulder, drivePos.shoulderPosition, drivePos.accelerationMax, drivePos.velocityMax);
                if (isServoAtPosition(shoulder, drivePos.shoulderPosition, EvoWork.SERVO_TOLERANCE)) {
                    currentDriveState = DriveState.COMPLETED;
                }
                break;
            case COMPLETED:

                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentDriveState == DriveState.COMPLETED) {
            inDrivePos = false;
            currentDriveState = DriveState.IDLE;
            activeDrivePosition = null; // Reset the active position
        }
    }

    public void startDrive() {
        activeDrivePosition = new DrivePosition(EvoWork.SHOULDER_DRIVE, EvoWork.ELBOW_DRIVE, EvoWork.WRIST_DRIVE, EvoWork.LEFT_FINGER_GRIP, EvoWork.RIGHT_FINGER_GRIP, EvoWork.HIGH_ACC, EvoWork.HIGH_VEL);
        currentDriveState = DriveState.MOVING_ELBOW;
    }

    // State machine for intake
    private enum IntakeState { IDLE, MOVING_SHOULDER, MOVING_WRIST, MOVING_ELBOW, MOVING_CLAWS, START_MOVING, COMPLETED }
    private volatile IntakeState currentIntakeState = IntakeState.IDLE;
    private IntakePosition activeIntakePosition;

    private boolean inIntakePos = false;

    static class IntakePosition {
        double shoulderPosition;
        double elbowPosition;
        double wristPosition;
        double leftFingerPosition;
        double rightFingerPosition;
        double accelerationMax;
        double velocityMax;

        public IntakePosition(double shoulderPos, double elbowPos, double wristPos,
                              double leftFingerPos, double rightFingerPos,
                              double accMax, double velMax) {
            this.shoulderPosition = shoulderPos;
            this.elbowPosition = elbowPos;
            this.wristPosition = wristPos;
            this.leftFingerPosition = leftFingerPos;
            this.rightFingerPosition = rightFingerPos;
            this.accelerationMax = accMax;
            this.velocityMax = velMax;
        }
    }

    private void handleIntakeSequence(ArmController.IntakePosition intakePos) {
        switch (currentIntakeState) {
            case START_MOVING:
            case MOVING_SHOULDER:
                // Move the shoulder to intake position
                moveServoWithTrapezoidalVelocity(shoulder, intakePos.shoulderPosition, intakePos.accelerationMax, intakePos.velocityMax);
                if (isServoAtPosition(shoulder, intakePos.shoulderPosition, EvoWork.SERVO_TOLERANCE)) {
                    currentIntakeState = IntakeState.MOVING_WRIST;
                }
                break;
            case MOVING_WRIST:
                // Move the wrist to intake position
                //moveServoGradually(wrist, intakePos.wristPosition);
                wrist.setPosition(intakePos.wristPosition);
                //moveServoWithTrapezoidalVelocity(wrist, intakePos.wristPosition, intakePos.accelerationMax, intakePos.velocityMax);
                if (isServoAtPosition(wrist, intakePos.wristPosition, EvoWork.SERVO_TOLERANCE)) {
                    currentIntakeState = IntakeState.MOVING_ELBOW;
                }
                break;
            case MOVING_ELBOW:
                // Move the elbow to intake position so it don't slap the floor
                // moveServoWith(elbow, intakePos.elbowPosition);
                moveServoWithTrapezoidalVelocity(elbow, intakePos.elbowPosition, intakePos.accelerationMax, intakePos.velocityMax);
                if (isServoAtPosition(elbow, intakePos.elbowPosition, EvoWork.SERVO_TOLERANCE)) {
                    // Check if the elbow is 70% down and open the claws if it is
                    currentIntakeState = IntakeState.MOVING_CLAWS;
                }
                break;
            case MOVING_CLAWS:
                leftFinger.setPosition(intakePos.leftFingerPosition);
                rightFinger.setPosition(intakePos.rightFingerPosition);
                    currentIntakeState = IntakeState.COMPLETED;
                break;
            case COMPLETED:
                // Sequence complete, reset the state or perform additional actions
                break;
        }
        // Check to reset the state to IDLE outside the switch
        if (currentIntakeState == IntakeState.COMPLETED) {
            inIntakePos = true;
            currentIntakeState = IntakeState.IDLE;
            activeIntakePosition = null; // Reset the active position
        }
    }

    public void StartIntake() {
        activeIntakePosition = new IntakePosition(EvoWork.SHOULDER_DRIVE, EvoWork.ELBOW_INTAKE, EvoWork.WRIST_INTAKE, EvoWork.LEFT_FINGER_GRIP, EvoWork.RIGHT_FINGER_GRIP, EvoWork.HIGH_ACC, EvoWork.HIGH_VEL);
        currentIntakeState = IntakeState.MOVING_SHOULDER;
    }

    public void startTopTwoBlueIntake() {
        activeIntakePosition = new IntakePosition(EvoWork.SHOULDER_INTAKE, EvoWork.ELBOW_TOP_TWO, EvoWork.WRIST_TOP_TWO, EvoWork.RIGHT_FINGER_INTAKE, EvoWork.LEFT_FINGER_GRIP, EvoWork.HIGH_ACC, EvoWork.HIGH_VEL);
        currentIntakeState = IntakeState.MOVING_SHOULDER;
    }
    public void startTopTwoRedIntake() {
        activeIntakePosition = new IntakePosition(EvoWork.SHOULDER_INTAKE, EvoWork.ELBOW_TOP_TWO, EvoWork.WRIST_TOP_TWO, EvoWork.RIGHT_FINGER_GRIP, EvoWork.LEFT_FINGER_INTAKE, EvoWork.HIGH_ACC, EvoWork.HIGH_VEL);
        currentIntakeState = IntakeState.MOVING_SHOULDER;
    }



    // Target positions for each servo
    private boolean isServoAtPosition(Servo servo, double targetPosition, double tolerance) {
        double currentPosition = servo.getPosition();
        double normalizedTarget = Range.clip(targetPosition, 0.0, 1.0); // Ensure target is within valid range
        double normalizedCurrent = Range.clip(currentPosition, 0.0, 1.0); // Ensure current position is within valid range

        return Math.abs(normalizedCurrent - normalizedTarget) < tolerance;
    }

    // move servo with ramping and soft-start created from the math and methods explained here: https://www.instructables.com/Servo-Ramping-and-Soft-Start/
    private void moveServoWithTrapezoidalVelocity(Servo servo, double targetPosition, double amax, double vmax) {
        double currentPosition = servo.getPosition();
        double currentVelocity = 0.0; // Initial velocity is zero
        double p0 = 0.0; // Distance needed to come to rest

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (Math.abs(currentPosition - targetPosition) > EvoWork.SERVO_TOLERANCE && opModeActive) {
            // Calculate the elapsed time since the last update
            double elapsedTime = timer.seconds();
            timer.reset();

            // Calculate the distance needed to come to rest given the current velocity
            p0 = 2 * currentVelocity + Math.abs(currentVelocity) * currentVelocity / (2 * amax);

            // Calculate the desired velocity based on the position error
            double velocityError = targetPosition - currentPosition - p0;
            double sign = Math.signum(velocityError);
            double desiredVelocity = currentVelocity + sign * amax;
            desiredVelocity = Range.clip(desiredVelocity, -vmax, vmax); // Constrain to vmax

            // Update the servo position based on the desired velocity and elapsed time
            currentPosition += desiredVelocity * elapsedTime;
            currentPosition = Range.clip(currentPosition, 0, 1); // Ensure the position is within valid range
            servo.setPosition(currentPosition);

            // Update the current velocity for the next iteration
            currentVelocity = desiredVelocity;

            // telemetry.addData("Servo Position", currentPosition);
            // telemetry.update();
        }
    }

    // moves servo gradually
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
    public boolean isActionCompleted() {
        // Check if each state machine is either in the COMPLETED state or IDLE
        boolean clawCompleted = (currentClawState == ClawState.COMPLETED || currentClawState == ClawState.IDLE);
        boolean scoreCompleted = (currentScoreState == ScoreState.COMPLETED || currentScoreState == ScoreState.IDLE);
        boolean driveCompleted = (currentDriveState == DriveState.COMPLETED || currentDriveState == DriveState.IDLE);
        boolean intakeCompleted = (currentIntakeState == IntakeState.COMPLETED || currentIntakeState == IntakeState.IDLE);

        return clawCompleted && scoreCompleted && driveCompleted && intakeCompleted;
    }
    public void stop() {
        running = false;
    }

    @Override
    public void run() {
        while (running) {
            if (!opModeActive) {
                // Handle the case when OpMode is no longer active
                break;
            }

            // Check and handle the current intake sequence
            if (activeIntakePosition != null) {
                handleIntakeSequence(activeIntakePosition);
            }
            // Check and handle the current drive sequence
            if (activeDrivePosition != null) {
                handleDriveSequence(activeDrivePosition);
            }
            // Check and handle the current score sequence
            if (activeScorePosition != null) {
                handleScoreSequence(activeScorePosition);
            }
            // Check and handle the current claw sequence
            if (activeClawPosition != null) {
                handleClawSequence(activeClawPosition);
            }
            // Small sleep to prevent this loop from consuming too much CPU
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

}