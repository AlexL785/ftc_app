package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
//Acesta e un test
/**
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a pushbot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "fullspeed", group = "Concept")
//@Disabled
public class full_speed extends LinearOpMode {

    static final int    CYCLE_MS    =   50;     // period of each cycle
    // Define class members
    private DcMotor leftMotor = null;
    private DcMotor leftMotor1 = null;
    private DcMotor rightMotor = null;
    private DcMotor rightMotor1 = null;
    private DcMotor throwMotor = null;
    double  power   = 1.0;
    //double throwPower = 0.5;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        leftMotor1 = hardwareMap.dcMotor.get("left_drive1");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightMotor1 = hardwareMap.dcMotor.get("right_drive1");
        //throwMotor = hardwareMap.dcMotor.get("throw_drive");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        //leftMotor1.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //rightMotor1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        //throwMotor.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

            // Display the current value
            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the motor to the new power and pause;
            leftMotor.setPower(power);
            leftMotor1.setPower(power);
            rightMotor.setPower(power);
            rightMotor1.setPower(power);
            //throwMotor.setPower(throwPower);
            sleep(CYCLE_MS);
            idle();
        }

        // Turn off motor and signal done;
        leftMotor.setPower(0);
        leftMotor1.setPower(0);
        rightMotor.setPower(0);
        rightMotor1.setPower(0);
        throwMotor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
