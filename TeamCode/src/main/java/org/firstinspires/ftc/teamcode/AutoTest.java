package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous: Iterative OpMode", group="Iterative Opmode") // @Autonomous(...) is the other common choice
@Disabled
public class AutoTest extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor backRightMotor, backLeftMotor;
    DcMotor frontRightMotor, frontLeftMotor;
    DcMotorController frontWheels, backWheels;


    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontWheels = hardwareMap.dcMotorController.get("frontWheels");
        backWheels = hardwareMap.dcMotorController.get("backWheels");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        runtime.reset();
    }

    @Override
    public void loop()
    {
        //telemetry.addData("Status", "Running: " + runtime.toString());
        if(runtime.seconds() < 3.0)
        {
            backLeftMotor.setPower(1);
            frontRightMotor.setPower(-1);
            //lag is here
            frontLeftMotor.setPower(1);
            backRightMotor.setPower(-1);
        }
        else
        {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
        }


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {

    }


    public void driveCircle()
    {

    }

}