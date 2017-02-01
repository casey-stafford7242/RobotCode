package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name="RPMFinder", group="TEST")
public class FindMaxRPM extends OpMode
{
    double speedCheckStartTime;
    int currentCounts;
    DcMotor leftFlywheel, rightFlywheel;
    boolean speedCheckTrigger = false;

    @Override
    public void init()
    {
        leftFlywheel = hardwareMap.dcMotor.get("leftShootMotor");
        rightFlywheel = hardwareMap.dcMotor.get("rightShootMotor");
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop()
    {
        leftFlywheel.setPower(1);
        rightFlywheel.setPower(1);
        telemetry.addData("Left Flywheel Speed", motorSpeed(leftFlywheel));
        telemetry.addData("Right Flywheel Speed", motorSpeed(rightFlywheel));
    }

    public int motorSpeed(DcMotor motor)
    {
        if(speedCheckTrigger == false)
        {
            speedCheckStartTime = System.nanoTime();
            currentCounts = motor.getCurrentPosition();
            speedCheckTrigger = true;
        }
        if(System.nanoTime() <= speedCheckStartTime + 100000000)
        {
            currentCounts = motor.getCurrentPosition() - currentCounts;
        }
        else
        {
            return currentCounts;
        }
        return 0;
    }

}