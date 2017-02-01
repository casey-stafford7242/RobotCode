package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;


@TeleOp(name="The God of This World: Iterative", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class TheDadesOfOurLives extends OpMode
//////////////////////COPY AND PASTE IS SIN////////////////////////////////////////////////////////

{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor aijdlkjsadlkasjdlkasjdlkasjdlkadjlkasjdlkasdjlkasdj2Casey;

//////////////////////COPY AND PASTE IS SIN//////////////////////////////

    @Override
    public void init()
    {
        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
    }

//////////////////////COPY AND PASTE IS SIN////////////////////////////////////////////////////////
//FOR FUTURE REFERENCE: telemetry.addData("example", "example");

    @Override
    public void loop()
    {
        //the right and left bumper is not engaged
        if(gamepad1.right_bumper == false && gamepad1.right_bumper == false)
        {
            if(gamepad1.left_stick_y != 0)
            {
                frontRight.setPower(gamepad1.left_stick_y);
                frontLeft.setPower(gamepad1.left_stick_y);
                backRight.setPower(gamepad1.left_stick_y);
                backLeft.setPower(gamepad1.left_stick_y);
            }
            //strafe right
            else if(gamepad1.left_stick_x > 0)
            {
                frontLeft.setPower(gamepad1.left_stick_x);
                frontRight.setPower(-gamepad1.left_stick_x);
                backRight.setPower(gamepad1.left_stick_x);
                backLeft.setPower(-gamepad1.left_stick_x);
            }
            //strafe left
            else if(gamepad1.left_stick_x < 0)
            {
                frontLeft.setPower(-gamepad1.left_stick_x);
                backLeft.setPower(gamepad1.left_stick_x);
                frontRight.setPower(gamepad1.left_stick_x);
                backRight.setPower(-gamepad1.left_stick_x);
            }
            //turn right
            else if(gamepad1.right_stick_x > 0)
            {
                frontLeft.setPower(gamepad1.left_stick_x);
                backLeft.setPower(gamepad1.left_stick_x);
                frontRight.setPower(-gamepad1.left_stick_x);
                backRight.setPower(-gamepad1.left_stick_x);
            }
            //turn left
            else if(gamepad1.right_stick_x < 0)
            {
                frontLeft.setPower(-gamepad1.right_stick_x);
                backLeft.setPower(-gamepad1.right_stick_x);
                frontRight.setPower(gamepad1.right_stick_x);
                backRight.setPower(gamepad1.right_stick_x);
            }
        }
        //the right bumper is engaged
        if(gamepad1.right_bumper == true)
        {
            //right diagonal
            if(gamepad1.left_stick_y > 0)
            {
                frontLeft.setPower(gamepad1.left_stick_y);
                backRight.setPower(gamepad1.left_stick_y);
                frontRight.setPower(0);
                backLeft.setPower(0);
            }
            //right reverse diagonal
            if(gamepad1.left_stick_y < 0)
            {
                frontLeft.setPower(gamepad1.left_stick_y);
                backRight.setPower(gamepad1.left_stick_y);
                frontRight.setPower(0);
                backLeft.setPower(0);
            }
        }
        //the left bumper is engaged
        if(gamepad1.left_bumper == true)
        {
            //left diagonal
            if(gamepad1.left_stick_y > 0)
            {
                frontRight.setPower(gamepad1.left_stick_y);
                backLeft.setPower(gamepad1.left_stick_y);
                backRight.setPower(0);
                frontLeft.setPower(0);
            }
            //left reverse diagonal
            if(gamepad1.left_stick_y < 0)
            {
                frontRight.setPower(gamepad1.left_stick_y);
                backLeft.setPower(gamepad1.left_stick_y);
                backRight.setPower(0);
                frontLeft.setPower(0);
            }
        }
    }
}
//SHE IS DONE
//SUICIDE
//////////////////////COPY AND PASTE IS SIN////////////////////////////////////////////////////////
