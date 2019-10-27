package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "ObjectDetectionAutonomous", group = "boundless")

public class ObjectDetectionAutonomous extends LinearOpMode {

    private DcMotor spool = null;       // Motor to control the up/down spool
    private DcMotor frontLeft = null;   // Motors to control the drive train
    private DcMotor frontRight = null;
    private DcMotor claw = null;   // Motor to control the horizontal motion

    BNO055IMU imu;
    Orientation angles;
    private double start_heading;       // The gyro reading at start

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AcCNMSn/////AAABmZxtapcFzUrfuuBkjremX+cFamDfj14dNrskO" +
                                              "/OerUlO+nD07sXXlI91OEZOE97gtnBnTtd6Z8bnBtXPfYIslrip9" +
                                              "bvDCyM6ywwAXrbyt7u6IkpgfE3Jf3MAug818wFFPfJn/MSZvWyU7" +
                                              "Jm3ALlyMXgycgbcgT+sGVVsoIpfgQh0QRc+MAkb2EHcPl4AOGr3cp" +
                                              "HMviz3b9k9JF6vzE8G23kxxYoGFrBpXFfBBilK6x6XDh+rfr5t+ca" +
                                              "CuM0k1wkpeR10tSSzfTx8fUpQllECZDjKQDOxGquiSYi/Kz/rdheB" +
                                              "AdIy5o9gu071JPF0HFxeIMQb6c1NcregbiAfUueFoyD/yty+CZnE9" +
                                              "sd0IVR3Cf0s";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
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
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // what's the start heading?
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        start_heading = angles.firstAngle;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        spool = hardwareMap.get(DcMotor.class, "spool");
        claw = hardwareMap.get(DcMotor.class, "claw");

        // reset encoder count kept by left motor.
        frontLeft.setTargetPosition(0);
        frontRight.setTargetPosition(0);

        // set left motor to run to target encoder position and stop with brakes on.
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set right motor to reverse and run to target encoder position and stop with brakes on
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first.
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

//        if (isSkystoneDetected()) {
//            stepsAfterDetection();
//            break;
//        }

        int increment = 200;
        int targetPositionLeft = frontLeft.getCurrentPosition() + increment;
        int targetPositionRight = frontRight.getCurrentPosition() + increment;

        frontLeft.setTargetPosition(targetPositionLeft);
        frontRight.setTargetPosition(targetPositionRight);

        frontLeft.setPower(0.1);
        frontRight.setPower(0.1);


        while (frontLeft.getCurrentPosition() > (-1 * increment) && frontRight.getCurrentPosition() > (-1 * increment)) {
            telemetry.addData("Encoder-Left: ", frontLeft.getCurrentPosition() + "  < " + targetPositionLeft);
            telemetry.addData("Encoder-Right: ", frontRight.getCurrentPosition() + "  < " + targetPositionRight);
            telemetry.update();
            idle();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);

        telemetry.addData("Current Position - Left: ", frontLeft.getCurrentPosition() + " Right: " + frontRight.getCurrentPosition());
        telemetry.update();

        try {
            Thread.sleep(20000);
        } catch (InterruptedException ex) {
            telemetry.addLine("Sleep was interrupted");
        }

        tfod.shutdown();
    }

    private boolean isSkystoneDetected() {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());

            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                if(recognition.getLabel() == "Skystone") {
                    return true;
                }
            }
            telemetry.update();
        }
        return false;
    }

    private void stepsAfterDetection() {




        gyroEncoderTurn(start_heading - 90);
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeft.setPower(0.2);
//        frontRight.setPower(0.2);
//        frontLeft.setTargetPosition(2000);
//        frontRight.setTargetPosition(2000);


    }

    public void gyroEncoderTurn(double targetAngle) {
        double error;
        int newLeftTarget;
        int newRightTarget;
        double rotPow;
        double speedScale = 0.5;

        for (int idx = 0; idx < 15; idx++) {

            // Determine how much our heading is off.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = targetAngle - angles.firstAngle;

            // Get out of loop early if there is no error.
            if (error == 0) { break; }
            // Determine new target position.
            newLeftTarget = frontLeft.getCurrentPosition() + (int) (error / 360 * 4000);
            newRightTarget = frontRight.getCurrentPosition() - (int) (error / 360 * 4000);

            //new target calculated
            frontLeft.setTargetPosition(newLeftTarget);
            frontRight.setTargetPosition(newRightTarget);

            // Power will also be a function of error. Big error = fast, small error = slow
            if (Math.abs(error) > 50) {
                rotPow = 0.8 * speedScale;
            } else if (Math.abs(error) > 40) {
                rotPow = 0.6 * speedScale;
            } else if (Math.abs(error) > 30) {
                rotPow = 0.4 * speedScale;
            } else if (Math.abs(error) > 20) {
                rotPow = 0.3 * speedScale;
            } else if (Math.abs(error) > 10) {
                rotPow = 0.2 * speedScale;
            } else {
                rotPow = 0.1 * speedScale;
            }

            // Do the incremental turn.
            frontLeft.setPower(rotPow);
            frontRight.setPower(rotPow);

            while (frontLeft.isBusy() && frontRight.isBusy()) {
            }

            // Stop all motion. This also applies the brakes.
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

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
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
