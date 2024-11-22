package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Servo Program", group="Linear Opmode")
public class Servo extends LinearOpMode {

    private CRServo extendIntake;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        extendIntake = hardwareMap.get(CRServo.class, "extendIntake");

        waitForStart();
        while (opModeIsActive() && runtime.milliseconds() < 1000) { // Here has to wait 0.5 second to let upper roller to drop down.
        }
        telemetry.addData("moved", extendIntake);
        extendIntake.setPower(100);
        while (opModeIsActive() && runtime.milliseconds() < 3000) { // Here has to wait 0.5 second to let upper roller to drop down.
        }

    }
}
