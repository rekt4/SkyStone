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


    public void runOpMode(){

        // This connects the software to the hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        spool = hardwareMap.get(DcMotor.class, "spool");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        clawLeft = hardwareMap.get(DcMotor.class, "clawLeft");
        clawRight = hardwareMap.get(DcMotor.class, "clawRight");

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
        double leftClawPowerRequest = 0;
        double rightClawPowerRequest = 0;
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        clawLeft.setDirection((DcMotor.Direction.REVERSE));
        while(opModeIsActive()){

            //linear slide moves up and down
            spoolPowerRequest = -this.gamepad2.left_stick_y;
            spool.setPower(spoolPowerRequest);

            //make the left and right wheels of the claw move based on which button is pressed
           if (this.gamepad2.a) {
               clawLeft.setPower(0.7);
               clawRight.setPower(0.7);
            } else if (this.gamepad2.b){
               clawLeft.setPower(-0.3);
               clawRight.setPower(-0.3);
            } else {
               clawLeft.setPower(0);
               clawRight.setPower(0);
           }


            drive = this.gamepad1.left_stick_y;
            strafe = this.gamepad1.left_stick_x;
            rotate = this.gamepad1.right_stick_x;

            frontLeft.setPower(drive + strafe + rotate);
            backLeft.setPower(drive - strafe + rotate);
            frontRight.setPower(drive - strafe - rotate);
            backRight.setPower(drive + strafe - rotate);

            // Code below helps for debugging
            telemetry.addData("Status", "Running");
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