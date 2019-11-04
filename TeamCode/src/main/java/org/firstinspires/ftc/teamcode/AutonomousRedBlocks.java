package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "AutonomousRedBlocks", group = "boundless")
public class AutonomousRedBlocks extends LinearOpMode {
    // Initiate all the motors. We have 5 motors

    // The spool controls the up/down motion
    private DcMotor spool = null;

    // The frontLeft and frontRight controls the drive train
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;

    // The leftRight controls the horizontal motion
    private DcMotor leftRight = null;

    // The frontBack controls the front-back claw
    private DcMotor frontBack = null;

    BNO055IMU imu;
    Orientation angles;
    private double start_heading;

    private ElapsedTime runtime = new ElapsedTime();

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU", and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // what's the start heading?
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        start_heading = angles.firstAngle;

        // This connects the software to the hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        spool = hardwareMap.get(DcMotor.class, "spool");
        leftRight = hardwareMap.get(DcMotor.class, "leftRight");
        frontBack = hardwareMap.get(DcMotor.class, "frontBack");

        // set left motor to run to target encoder position and stop with brakes on.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set right motor to reverse and run to target encoder position and stop with brakes on
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // This always updates telemetry
        telemetry.addData("Status", "Boundless Robotics Ready to Launch");
        telemetry.update();

        waitForStart();

        // Step 1: Open the Claw and lift the spool
        openClaw(0.5, 0.8);
        spoolUp(0.5, 2.0);

        // Step 2: Drive forward by 40 inches
        driveUsingEncoder(0.3, -40, -40, 12);

        // Step 3: Lower the spool & claw; Be careful here because the spool may have dropped a bit
        //         because of the weight.
        spoolDown(0.7, 1);
        closeClaw(0.3, 1);

        // Step 4: Pull the robot & the block back
        driveUsingEncoder(0.3, 42, 42, 12);

        // Step 5: Move forward by a bit - so we can get a proper turn & turn by 90 degrees
        driveUsingEncoder(0.1, -12, -12, 5);
        turnRightUsingEncoder(0.3, 78.0, 7);

        // Step 6: Drive across the bridge, and drop off the block.
        driveUsingEncoder(0.75, -50, -50, 15);

        // Step 7: Drop off the block.
        openClaw(0.5, 0.2);
        spoolUp(0.5, 2.0);
        driveUsingEncoder(0.1, 4,4,1);

        closeClaw(0.5, 0.5);
        spoolDown(0.5, 0.8);

        // Step 8: Drive back all the way to the wall.
        driveUsingEncoder(0.75, 110, 110, 15);

        // Step 9: Turn to get another block.
        driveUsingEncoder(0.3, 4, 4, 3);
        turnLeftUsingEncoder(0.3, 0, 7);
        printCurrentAngle();
        driveUsingEncoder(0.5,12,12,5);

        // Step 10: Open the claw and raise the spool.
        openClaw(0.5, 0.8);
        spoolUp(0.5, 2.2);

        // Step 11: Go forward and pick up a block.
        driveUsingEncoder(0.3, -40, -40, 12);
        spoolDown(0.7, 1);
        closeClaw(0.3, 1);

        // Step 12: Drive back with the block.
        driveUsingEncoder(0.3, 42, 42, 12);

        // Step 13: Turn left to deposit second block.
        driveUsingEncoder(0.1, -12, -12, 5);
        turnLeftUsingEncoder(0.3, 78.0, 7);
    }

    public void printCurrentAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Current Angle: ", angles.firstAngle);
        telemetry.update();
        try { Thread.sleep(2000); } catch(Exception ex) {}
    }

    /**
     * Utility method to move the robot using encoder values
     * @param newLeftTarget -- This is the Left wheel's target position.
     * @param newRightTarget
     * @param speed
     * @param timeoutSeconds
     */
    public void moveUsingEncoder(int newLeftTarget, int newRightTarget, double speed, double timeoutSeconds) {
        //new target calculated
        frontLeft.setTargetPosition(newLeftTarget);
        frontRight.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutSeconds) &&
                (frontLeft.isBusy() && frontRight.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition());
            telemetry.update();
        }
        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);

        // Turn off RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     *
     * @param speed
     * @param targetAngle
     * @param timeoutSeconds
     */
    public void turnLeftUsingEncoder(double speed, double targetAngle, double timeoutSeconds) {
        double turnRemaining;
        int newLeftTarget;
        int newRightTarget;
        double rotPow;

        for (int idx = 0; idx < 15; idx++) {
            // Determine how much our heading is off.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            turnRemaining = targetAngle - angles.firstAngle;

            // Get out of loop early if there is no turnRemaining.
            if (turnRemaining <= 0) {
                break;
            }

            telemetry.addData("Turn Remaining", "" + turnRemaining);
            telemetry.update();

            // Determine new target position.
            newLeftTarget = frontLeft.getCurrentPosition() - (int) ((turnRemaining / 360) * 2880);
            newRightTarget = frontRight.getCurrentPosition() + (int) ((turnRemaining / 360) * 2880);


            // Power will also be a function of turnRemaining. Big turnRemaining = fast, small turnRemaining = slow
            if (Math.abs(turnRemaining) > 50) {
                rotPow = 0.8 * speed;
            } else if (Math.abs(turnRemaining) > 40) {
                rotPow = 0.6 * speed;
            } else if (Math.abs(turnRemaining) > 30) {
                rotPow = 0.4 * speed;
            } else if (Math.abs(turnRemaining) > 20) {
                rotPow = 0.3 * speed;
            } else if (Math.abs(turnRemaining) > 10) {
                rotPow = 0.2 * speed;
            } else {
                rotPow = 0.1 * speed;
            }
            moveUsingEncoder(newLeftTarget, newRightTarget, rotPow, timeoutSeconds);
        }

    }

    /**
     *
     * @param speed
     * @param targetAngle
     * @param timeoutSeconds
     */
    public void turnRightUsingEncoder(double speed, double targetAngle, double timeoutSeconds) {
        double turnRemaining;
        int newLeftTarget;
        int newRightTarget;
        double rotPow;

        for (int idx = 0; idx < 15; idx++) {
            // Determine how much our heading is off.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            turnRemaining = targetAngle + angles.firstAngle;

            // Get out of loop early if there is no turnRemaining.
            if (turnRemaining <= 0) {
                break;
            }

            telemetry.addData("Turn Remaining", "" + turnRemaining);
            telemetry.update();

            // Determine new target position.
            newLeftTarget = frontLeft.getCurrentPosition() + (int) ((turnRemaining / 360) * 2880);
            newRightTarget = frontRight.getCurrentPosition() - (int) ((turnRemaining / 360) * 2880);


            // Power will also be a function of turnRemaining. Big turnRemaining = fast, small turnRemaining = slow
            if (Math.abs(turnRemaining) > 50) {
                rotPow = 0.8 * speed;
            } else if (Math.abs(turnRemaining) > 40) {
                rotPow = 0.6 * speed;
            } else if (Math.abs(turnRemaining) > 30) {
                rotPow = 0.4 * speed;
            } else if (Math.abs(turnRemaining) > 20) {
                rotPow = 0.3 * speed;
            } else if (Math.abs(turnRemaining) > 10) {
                rotPow = 0.2 * speed;
            } else {
                rotPow = 0.1 * speed;
            }
            moveUsingEncoder(newLeftTarget, newRightTarget, rotPow, timeoutSeconds);
        }

    }

    public void driveUsingEncoder(double speed, double leftInches, double rightInches, double timeoutSeconds) {

        final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 0.25;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (!opModeIsActive()) {
            return;
        }

        // Determine new target position, and pass to motor controller
        newLeftTarget = frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        moveUsingEncoder(newLeftTarget, newRightTarget, speed, timeoutSeconds);
    }

    public void spoolUp(double speed, double timeoutSeconds) {
        long timeoutInMillis = (long) (timeoutSeconds * 1000);
        spool.setPower(speed);
        try { Thread.sleep(timeoutInMillis); } catch(Exception ex) {}
        spool.setPower(0);
    }

    public void spoolDown(double speed, double timeoutSeconds) {
        long timeoutInMillis = (long) (timeoutSeconds * 1000);
        spool.setPower(-1 * speed);
        try { Thread.sleep(timeoutInMillis); } catch(Exception ex) {}
        spool.setPower(0);
    }

    public void openClaw(double speed, double timeoutSeconds) {
        frontBack.setPower(speed);
        long timeoutInMillis = (long) (timeoutSeconds * 1000);
        try { Thread.sleep(timeoutInMillis); } catch(Exception ex) {}
        frontBack.setPower(0);
    }

    public void closeClaw(double speed, double timeoutSeconds) {
        frontBack.setPower(-1 * speed);
        long timeoutInMillis = (long) (timeoutSeconds * 1000);
        try { Thread.sleep(timeoutInMillis); } catch(Exception ex) {}
        frontBack.setPower(0);
    }
}
