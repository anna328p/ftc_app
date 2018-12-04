package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;

@Autonomous(name = "TestOpMode (Blocks to Java)", group = "")
public class TestOpMode extends LinearOpMode {

    private AndroidSoundPool androidSoundPool;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        androidSoundPool = new AndroidSoundPool();

        // Put initialization blocks here.
        androidSoundPool.initialize(SoundPlayer.getInstance());
//	androidSoundPool.preloadSound("SovietAnthem.mp3");
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            androidSoundPool.play("SovietAnthem.mp3");
        }

        androidSoundPool.close();
    }
}
