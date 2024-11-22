package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;
@Autonomous(name="Cascade Program", group="Linear Opmode")
public class CascadeProgram extends LinearOpMode {

    private DcMotor cascade;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor cascadeA;
    private DcMotor cascadeB;

    @Override
    public void runOpMode() throws InterruptedException {
        cascade = hardwareMap.dcMotor.get("cascade");
        cascade.setDirection(DcMotorSimple.Direction.REVERSE);
        cascade.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascade.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        cascadeA = hardwareMap.dcMotor.get("cascadeLeft");
        cascadeB = hardwareMap.dcMotor.get("cascadeRight");
        cascadeA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascadeB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascadeA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascadeB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascadeB.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive() && runtime.milliseconds() < 1000) { // Here has to wait 0.5 second to let upper roller to drop down.
        }
        telemetry.addData("Cascade", cascade);
//        cascade_move(cascade,0.5,5,5);
        side_cascade_move(cascadeA, cascadeB, 0.5, 3, 2);

    }

    public void side_cascade_move(DcMotor cascadeA, DcMotor cascadeB, double speed, double run_inches, int timeoutS) {
        double COUNTS_PER_MOTOR_REV = 1440;    // Normally, Motor Encoder has 1440 counts per rotation
        double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
        double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415); // This is how many counts for encoder if the wheel runs 1 inche.
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            int target = (int) (run_inches * COUNTS_PER_INCH);
            cascadeA.setTargetPosition(target);
            cascadeB.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            cascadeA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            cascadeB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            cascadeA.setPower(Math.abs(speed));
            cascadeB.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (cascadeA.isBusy() || cascadeB.isBusy() )) {
                // Display it for the driver.
                telemetry.addData("Target :",  "encoder counts %7d: timeout       = %3d", target, timeoutS);
                telemetry.update();
            }
            // Stop all motion;
            // cascade.setPower(0);
            telemetry.addData("Stopped :",  "encoder counts %7d: timeout       = %3d", target, timeoutS);
            telemetry.update();
        }
    }

        public void cascade_move(DcMotor motor, double speed,
        double run_inches,
        int timeoutS) {
            double     COUNTS_PER_MOTOR_REV    = 1440 ;    // Normally, Motor Encoder has 1440 counts per rotation
            double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
            double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
            double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415); // This is how many counts for encoder if the wheel runs 1 inche.

            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                int target = (int)(run_inches * COUNTS_PER_INCH);
                motor.setTargetPosition(target);

                // Turn On RUN_TO_POSITION
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                motor.setPower(Math.abs(speed));

                // keep looping while we are still active, and if there is time left, and motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, or time out, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion if blocked by others.
                // However, if you require that BOTH motor have finished their moves and time is out, before the robot continues
                // onto the next step, use (isBusy() || < timeOutS) in the loop test.
                while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motor.isBusy() )) {
                    // Display it for the driver.
                    telemetry.addData("Target :",  "encoder counts %7d: timeout       = %3d", target, timeoutS);
                    telemetry.addData("Current:",  "encoder counts %7d: current_time= %3d", motor.getCurrentPosition(), (int) runtime.seconds());
                    telemetry.update();
                }
                // Stop all motion;
                // cascade.setPower(0);
                telemetry.addData("Stopped :",  "encoder counts %7d: timeout       = %3d", target, timeoutS);
                telemetry.update();
                //  sleep(250);   // optional pause after each move
            }
        }
    }

