package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;


@TeleOp(name="DadE: Iterative", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled
public class TheyCallHimDadE extends OpMode
{
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;

    @Override
    public void init()
    {
        backLeftMotor = hardwareMap.dcMotor.get("left_drive");
    }

    @Override
    public void loop()
    {

        if(gamepad1.left_stick_y  != 0)
        {
            if(gamepad1.right_stick_x == 0)
            {
                frontRightMotor.setPower(gamepad1.left_stick_y);
                backRightMotor.setPower(gamepad1.left_stick_y);
                frontLeftMotor.setPower(gamepad1.left_stick_y);
                backLeftMotor.setPower(gamepad1.left_stick_y);
            }
            else if(gamepad1.right_stick_x > 0)
            {
                frontRightMotor.setPower(gamepad1.left_stick_y);
                backRightMotor.setPower(gamepad1.left_stick_y);
                frontLeftMotor.setPower(gamepad1.right_stick_x);
                backLeftMotor.setPower(-gamepad1.right_stick_x);
            }
            else if(gamepad1.right_stick_x < 0)
            {
                frontRightMotor.setPower(gamepad1.right_stick_x);
                backRightMotor.setPower(-gamepad1.right_stick_x);
                frontLeftMotor.setPower(gamepad1.left_stick_y);
                backLeftMotor.setPower(gamepad1.left_stick_y);
            }
        }
        else
        {
            if(gamepad1.right_stick_x == 0)
            {
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
            }
            else if(gamepad1.right_stick_x > 0)
            {
                frontRightMotor.setPower(gamepad1.right_stick_x);
                backRightMotor.setPower(-gamepad1.right_stick_x);
                frontLeftMotor.setPower(-gamepad1.right_stick_x);
                backLeftMotor.setPower(gamepad1.right_stick_x);
            }
            else if (gamepad1.right_stick_x < 0)
            {
                frontRightMotor.setPower(-gamepad1.right_stick_x);
                frontLeftMotor.setPower(gamepad1.right_stick_x);
                backLeftMotor.setPower(-gamepad1.right_stick_x);
                backRightMotor.setPower(gamepad1.right_stick_x);
            }
        }
    }


}
