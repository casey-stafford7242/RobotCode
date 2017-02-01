/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */ 

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;

@TeleOp(name="TELETOPETPEOTPEWOT: Iterative OpMode", group="Iterative Opmode")
@Disabled
public class CustomTeleOp extends OpMode
{
    DcMotor backRightMotor, backLeftMotor;
    DcMotor frontRightMotor, frontLeftMotor;
    DcMotorController frontWheels, backWheels;



    @Override
	public void init()
    {
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        frontWheels = hardwareMap.dcMotorController.get("frontWheels");
        backWheels = hardwareMap.dcMotorController.get("backWheels");
    }


	@Override
	public void loop()
    {
        telemetry(backLeftMotor, frontLeftMotor, backRightMotor, frontRightMotor);

        if(gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_y == 0)
        {
            backRightMotor.setPower(-gamepad1.left_trigger);
            backLeftMotor.setPower(gamepad1.left_trigger);
            frontRightMotor.setPower(-gamepad1.left_trigger);
            frontLeftMotor.setPower(gamepad1.left_trigger);
        }
        else if(gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_y == 0)
        {
            backRightMotor.setPower(gamepad1.right_trigger);
            backLeftMotor.setPower(-gamepad1.right_trigger);
            frontRightMotor.setPower(gamepad1.right_trigger);
            frontLeftMotor.setPower(-gamepad1.right_trigger);
        }
        else if(gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
        {
            backLeftMotor.setPower(gamepad1.left_stick_y);
            frontLeftMotor.setPower(gamepad1.left_stick_y);
            backRightMotor.setPower(-gamepad1.right_stick_y);
            frontRightMotor.setPower(-gamepad1.right_stick_y);
        }
        else if(gamepad1.left_stick_y == 0 && gamepad1.right_stick_y == 0 && gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0)
        {
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
        }

    }

    public void telemetry(DcMotor backLeftMotor, DcMotor frontLeftMotor, DcMotor backRightMotor, DcMotor frontRightMotor)
    {
        telemetry.addData("BRMotor", backRightMotor.getPower());
        telemetry.addData("BLMotor", backLeftMotor.getPower());
        telemetry.addData("FLMotor", frontLeftMotor.getPower());
        telemetry.addData("FRMotor", frontRightMotor.getPower());
        telemetry.update();
    }

	@Override
	public void stop()
    {

    }

}


