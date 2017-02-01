package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.vuforia.HINT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@TeleOp(name="VuforiaTest: Iterative Opmode", group = "Iterative Opmode")
@Disabled
public class VuforiaTest extends OpMode
{
    VuforiaLocalizer vuforia;
    VuforiaTrackables trackables;
    VuforiaTrackable tools, wheels, gears, legos;
    VuforiaLocalizer.Parameters parameters;
    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;

    public void init()
    {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        parameters.vuforiaLicenseKey = "ASUFH4r/////AAAAGcn+CWeboURVnsdSRGRhcHVtHrLwXdseP/mllNscLs7e203vQlxq213cp9EmpkZ7UhvSsrB0aPIGkSaLMVMjOPmnPCcsQLjBpwJvEO/1TxCttbE1LzU0Hf2hKBIqqM+Upw9ST1GZHDhkx4IWB64UvJJM+QAMDSVXPKUPBmx1rZ7fy0KDYm6FzXHradtZ3ZLIr8g8kewDL4Pu61DWvsd1jDkplnvCtzwCmNLf7u0fLZ7blTiE6srObAmkNSonW9MRLkjI8mQQ2316RBYax0EO9BOEcS4dwvYmX5XMCx4MW0Gk5EgSR7hCGwFnl7XV8PvfpcbeFnEwyWFoWT5/32Wp7RuYj6socuCTm+sXxp93aSrs";
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        trackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        if(trackables == null)
        {
            telemetry.addData("HALP", "IT Broken");
            return;
        }
        trackables.activate();
        tools = trackables.get(1);
        wheels = trackables.get(0);
        gears = trackables.get(3);
        legos = trackables.get(2);
        legos.setName("Legos");
        wheels.setName("Wheels");
        gears.setName("Gears");
        tools.setName("Tools");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

    }

    public void loop()
    {
        for(VuforiaTrackable trackable : trackables)
        {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)trackable.getListener()).getPose();
            if(pose != null)
            {
                telemetry.addData(trackable.getName() + " Translation", pose.getTranslation());
            }
            else
            {
                telemetry.addData(trackable.getName() + " Translation", "Unknown");
            }
        }
    }

    /*
    public void loop()
    {


        if (robotLocationTransform != null)
        {
            previousLocation = robotLocationTransform;
        }
        if (previousLocation != null)
        {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));

        }
        else
        {
            telemetry.addData("Pos", "Unknown");
        }
        telemetry.update();
        */
//   }
    String format(OpenGLMatrix transformationMatrix)
    {
        VectorF translation = transformationMatrix.getTranslation();
        Orientation orientation = Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return String.format("%s %s", orientation.toString(), translation.toString());
    }

}
