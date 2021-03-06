package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.Callable;


@Autonomous(name = "AutoRedBlocksBridge", group = "boundless")
public class AutoRedBlocksBridge extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BoundlessRobot robot = new BoundlessRobot(hardwareMap, telemetry);

        Callable<Boolean> opMode = new Callable<Boolean> () {
            public Boolean call() {
                return opModeIsActive();
            }
        };

        // This always updates telemetry
        telemetry.addData("Status", "Boundless Robotics Ready to Launch");
        telemetry.update();

        waitForStart();

        // Step 1: Drive forward by 30 inches
        robot.driveUsingEncoder(telemetry, opMode,0.4, 32, 32, 10);

        try { Thread.sleep(100); } catch(Exception ex) {}

        // Step 2: Lower the rear servo and grab the block
        robot.rear.setPosition(0.4);
        try { Thread.sleep(500); } catch(Exception ex) {}

        // Step 3: Drive back a little
        robot.driveUsingEncoder(telemetry, opMode,0.6, -8, -8, 10);

        // Step 4: Turn left 90 degrees
        robot.turnUsingEncoder(telemetry, opMode, 0.7, 90, false, 3);

        // Step 5: Drive beyond the bridge
        robot.driveUsingEncoder(telemetry, opMode,0.5, 52, 52, 10);

        // Step 6: Lift the rear servo
        robot.rear.setPosition(1);
        try{ Thread.sleep(3000); } catch( Exception ex ) {}

        // Step 7: Drive under the bridge
        robot.driveUsingEncoder(telemetry, opMode,0.5, -12, -12, 10);
    }
}
