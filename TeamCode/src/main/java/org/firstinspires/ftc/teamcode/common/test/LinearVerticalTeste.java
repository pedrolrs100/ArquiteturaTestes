package org.firstinspires.ftc.teamcode.common.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
//import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V2;

@Disabled
@TeleOp(name = "TesteLinearSemPIDF")
public class LinearVerticalTeste extends OpMode {

    //private V2 robot;
    GamepadEx gamepadEx;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    DcMotorEx motorL;
    DcMotorEx motorR;




    @Override
    public void init() {
        this.motorL =  hardwareMap.get(DcMotorEx.class, HardwareNames.verticalL);
        this.motorR =  hardwareMap.get(DcMotorEx.class, HardwareNames.verticalR);
        this.motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad2.left_trigger > 0){
            motorR.setPower(gamepad2.left_trigger);
            motorL.setPower(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger > 0) {
            motorR.setPower(gamepad2.right_trigger * -1 );
            motorL.setPower(gamepad2.right_trigger * -1 );
        }

        else{
            motorR.setPower(0);
            motorL.setPower(0);
        }
    }
}
