package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.LinearVertical.SistemasLineares.Condicoes.CondicoesGeral;

public class SistemaLinear  {

    public DcMotorEx motor;
    public Controladorpidf controlador;
    public CondicoesGeral condicoesParada;
    private int ID;/*todo : não usado*/
    public static boolean isBusy;

    int position,targetPosition;



    public SistemaLinear(
        //DcMotorEx Motor,
        //Controladorpidf controlador,
        //CondicoesGeral parada
    ){
        //this.motor = Motor;
        //this.position = motor.getCurrentPosition();
        //this.targetPosition = this.position;
        //this.parada = parada;
        //this.controlador = controlador;
    }



    public Action GoTo(int target) {

        return new Action() {
            int id = target;
            ElapsedTime time = new ElapsedTime();
            boolean started = false;
            int topo = 2650;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    targetPosition = target;
                    time.reset();
                    started = true;
                    ID = id;
                    isBusy = true;

                }
                controlador.PIDF();
                if(condicoesParada.) return ;
                //PIDF();



                return true;
            }

        };

    }

    public void reset(DcMotorEx Motor) {
        Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = 0;
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
