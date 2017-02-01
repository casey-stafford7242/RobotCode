package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@TeleOp (name = "VuforiaAngleGettin'Testttttttt", group = "TestyMcTestorson")
//@Disabled
public class VuforiaAngleGettinTest extends OpMode
{
    VuforiaLocalizer vuforia;
    VuforiaTrackables trackables;
    VuforiaTrackable tools, wheels, gears, legos;
    VuforiaLocalizer.Parameters parameters;
    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;
    OpenGLMatrix pose;
    @Override
    public void init()
    {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        parameters.vuforiaLicenseKey = "AUwWjyD/////AAAAGaF2Q8VWyUK5rxcXs5WXlXdUrSLFdS8S590Xt+5aJetftc4zTLIt37B+G/1q4ChZ1lTeyvTtQtT9fsfiY1K35GGfa5+shUWwVR3gOCJ16Su4UEGaKWLkU/wOThsP9zX1UKGotIekohu+tiGAEJdNNoZuWf4vJNHxEoFzo15DI/AYHscmnRuSeUCA/41bgRk1NO4VMPYhh/BRlC1KcYDXsS1nDA5d1O51+znVNkzh3Z3RIv3VN330SXCe1EkTa6IUmAriYQJZgBQzC+SV3Y+/TvI94Yia3MKjmiL6g5SwOK5yV6OVr8FeTwmlhOlU+D40x8Nybo5iUnFRojFNwniFhgtNNugBE7tvPQ0zX1FH1Zmw";
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        trackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
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

    @Override
    public void loop()
    {
        if(gears == null)
        {
            telemetry.addData("Gears == null", "null");
            stop();
        }
        pose = ((VuforiaTrackableDefaultListener)gears.getListener()).getPose();
        if(pose.getTranslation() != null)
        {
            double degreesToTurn = Math.toDegrees(Math.atan2(pose.getTranslation().get(1), pose.getTranslation().get(2)));
            telemetry.addData("Degrees to TURN", degreesToTurn);
        }
    }
}
