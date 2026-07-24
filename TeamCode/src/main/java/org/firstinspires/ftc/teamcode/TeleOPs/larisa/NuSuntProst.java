package org.firstinspires.ftc.teamcode.TeleOPs.larisa;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="NuSuntProst") // ba da - alex mic

public class NuSuntProst extends LinearOpMode {

    DcMotor motorStangaFata = null;
    DcMotor motorDreaptaFata = null;
    DcMotor motorStangaSpate = null;
    DcMotor motorDreaptaSpate = null;

    @Override
    public void runOpMode() throws InterruptedException {

        motorDreaptaSpate = hardwareMap.get(DcMotor.class, "motorStF");
        motorDreaptaFata = hardwareMap.get(DcMotor.class, "motorDrF");
        motorStangaSpate = hardwareMap.get(DcMotor.class, "motorStS");
        motorStangaFata = hardwareMap.get(DcMotor.class, "motorDrS");
                    //AM UITAT TOT DE AMU 2 ANI BAG PULA DE LA MARIO
    }
}
