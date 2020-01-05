package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MecanumDrive", group="boundless")

public class MecanumDrive extends LinearOpMode {

    // Initiate all the motors. We have 4 motors

    // The spool controls the up/down motions
  //  private DcMotor spool = null;

    // The frontLeft and frontRight controls the drive train
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor clawLeft = null;
    private DcMotor clawRight = null;
    private DcMotor spool = null;
    private Servo rear = null;
    private Servo capstone = null;

    private double drive_scale = 1;
    private double DRIVE_POWER_HIGH_SCALE = .9;
    private double DRIVE_POWER_LOW_SCALE = 0.5;

    private boolean intake_is_spinning = false;
    private boolean is_strafing_left = false;
    private boolean is_strafing_right = false;



    public void runOpMode(){

        // This connects the software to the hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        spool = hardwareMap.get(DcMotor.class, "spool");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        clawLeft = hardwareMap.get(DcMotor.class, "clawLeft");
        clawRight = hardwareMap.get(DcMotor.class, "clawRight");
        rear = hardwareMap.get(Servo.class, "rear");
        capstone = hardwareMap.get(Servo.class, "capstone");

        // This always updates telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // This is sets the original joystick power
       // double spoolLeftStick=0;

        double drive = 0;
        double strafe = 0;
        double rotate = 0;
        double spoolPowerRequest = 0;
        double leftClawPowerRequest =  0;
        double rightClawPowerRequest = 0;
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        clawLeft.setDirection((DcMotor.Direction.REVERSE));
        while(opModeIsActive()){

            //linear slide moves up and down
            spoolPowerRequest = -this.gamepad2.left_stick_y;
            spool.setPower(spoolPowerRequest);

            //INTAKE: make the left and right wheels of the claw move based on which button is pressed
           if (this.gamepad2.a) {
               clawLeft.setPower(0.7);
               clawRight.setPower(0.7);
               intake_is_spinning = true;
            } else if (this.gamepad2.b){
               clawLeft.setPower(-0.3);
               clawRight.setPower(-0.3);
               intake_is_spinning = true;
            } else {
               clawLeft.setPower(0);
               clawRight.setPower(0);
               intake_is_spinning = false;
           }

           // STRAFING...
            if (this.gamepad1.left_bumper) {
                frontLeft.setPower(.6);
                backLeft.setPower(-.5);
                frontRight.setPower(-.6);
                backRight.setPower(.5);
                is_strafing_left = true;
                is_strafing_right = false;
            } else if (this.gamepad1.right_bumper) {
                frontLeft.setPower(-.6);
                backLeft.setPower(.5);
                frontRight.setPower(.6);
                backRight.setPower(-.5);
                is_strafing_left = false;
                is_strafing_right = true;
            } else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                is_strafing_left = false;
                is_strafing_right = false;
            }

            // DRIVING and ROTATION...
            // trying to reduce voltage drop
            // if intake is spinning, then reduce maximum drive power
            drive = this.gamepad1.left_stick_y;
            rotate = -1*this.gamepad1.right_stick_x;

            if (intake_is_spinning){
                drive_scale = DRIVE_POWER_LOW_SCALE;
            } else {
                drive_scale = DRIVE_POWER_HIGH_SCALE;
            }
            frontLeft.setPower(drive_scale*(drive + rotate));
            backLeft.setPower(drive_scale*(drive + rotate));
            frontRight.setPower(drive_scale*(drive - rotate));
            backRight.setPower(drive_scale*(drive - rotate));

            // rear grabber...
            if (gamepad2.y) {
                rear.setPosition(1);
            } else if (gamepad2.x) {
                rear.setPosition(0.4);
            } else if (gamepad2.right_trigger == 1) {
                rear.setPosition(0.8);
            }

            // capstone stick...
            if (gamepad2.left_trigger == 1) {
                capstone.setPosition(1);
            } else if (gamepad2.left_bumper) {
                capstone.setPosition(0);
            } else if (gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down) {
                capstone.setPosition(0.3);
            }

            // Code below helps for debugging
            telemetry.addData("Status", "Running");
            //telemetry.addData("rear grabber pos", rear.getPosition());
            //telemetry.addData("capstone pos", capstone.getPosition());
            telemetry.addData("intake_is_spinning", intake_is_spinning);
            telemetry.addData("is_strafing_left", is_strafing_left);
            telemetry.addData("is_strafing_right", is_strafing_right);
            telemetry.update();
        }
    }


    public void tank(double fbLeftStick, double fbRightStick){
        frontLeft.setPower(fbLeftStick);
        frontRight.setPower(fbRightStick);
        telemetry.addData("Motor Power - Left", frontLeft.getPower());
        telemetry.addData("Motor Power - Right", frontRight.getPower());

    }
}