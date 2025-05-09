package org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasSuperiores.BracoGarra;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.HashMap;

@Config
public class BracoGarraSuperior {
    int ID = 0;
    boolean isBusy = false;
    public static boolean monitor = false;
    public double servoBracoSuperiorPosition =0.659;
    public static double posicaoBracoTranser =  0.084;
    public Servo bracoGarraSuperiorServo;
    public BracoGarraSuperiorStates bracoGarraSuperiorState = BracoGarraSuperiorStates.READYTO_TRANSFER;
    public BracoGarraSuperiorStates lastStateBracoSuperior;
    public HashMap<BracoGarraSuperiorStates, Double> mapBracoSuperior = new HashMap<>();
    public static  double   maxAcc = 2700, maxVelocity  = 4400, distance = 0, tempo = 0;

    public static double bracoReadyToHangPos = 0, bracoHangPos = 0;

    public BracoGarraSuperior(HardwareMap hardwareMap, Telemetry telemetry) {
        bracoGarraSuperiorServo = hardwareMap.get(Servo.class, HardwareNames.bracoGarraSuperiorServo);

        mapBracoSuperior.put(BracoGarraSuperiorStates.BASKET, 0.72);///todo okey
        mapBracoSuperior.put(BracoGarraSuperiorStates.INITIAL,0.0928);//todo rever as posições
        mapBracoSuperior.put(BracoGarraSuperiorStates.READYTO_TRANSFER,  posicaoBracoTranser);//todo rever as posições
        mapBracoSuperior.put(BracoGarraSuperiorStates.TRANSFER,  posicaoBracoTranser);///todo okey
        mapBracoSuperior.put(BracoGarraSuperiorStates.OUTTAKE_EJECTING, 1.0);
        mapBracoSuperior.put(BracoGarraSuperiorStates.READY_TO_HANG, bracoReadyToHangPos);
        mapBracoSuperior.put(BracoGarraSuperiorStates.HANG, bracoHangPos);

        mapBracoSuperior.put(BracoGarraSuperiorStates.INTAKE,0.16777777777777775);//todo okey
        mapBracoSuperior.put(BracoGarraSuperiorStates.OUTTAKE_CHAMBER, 1.0);//todo okey

        mapBracoSuperior.put(BracoGarraSuperiorStates.READY_OUTTAKE,0.78);//todo rever as posições



    }


    public Action goToTransfer(){//todo okey
        final int IDaction = 3;
        return new InstantAction(()->{
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.TRANSFER;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
            lastStateBracoSuperior = bracoGarraSuperiorState;
        });
    }


    public Action goToReadyToTransfer(){//todo okey
        final int IDaction = 2;
        return new InstantAction(()->{
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.READYTO_TRANSFER;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
            lastStateBracoSuperior = bracoGarraSuperiorState;
        });
    }
    public Action timeActionBracoGarraSuperior(BracoGarraSuperiorStates targetState){
        return new Action(){
            private final int IDaction = 1;
            boolean FIRST = true;
            double delay = 0.2;
            ElapsedTime time = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if(FIRST) {
                    time.reset();
                    FIRST = false;
                    isBusy = true;
                    ID = IDaction;
                }

                if(ID != IDaction){
                    return false;
                }

                bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(targetState));
                if(delay <= time.time()){
                    bracoGarraSuperiorState = targetState;
                    isBusy = false;
                    return false;
                }
                return true;
            }
        };

    }
    public Action goToOuttakeBASKET(){//todo okey
        final int IDaction = 4;
        return new InstantAction(()->{
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.BASKET;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }
    public Action goToReadOuttakeCHAMBER(){
        final int IDaction = 5;
        return new InstantAction(() -> {
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.READY_OUTTAKE;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }

   public Action goToIntakeCHAMBER(){//todo okey
       final int IDaction = 6;
        return new InstantAction(() ->{
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.INTAKE;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
   }

    public Action goToInital(){//todo okey
        final int IDaction = 7;
        return new InstantAction(() -> {
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.INITIAL;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }

   public Action goToOuttakeCHAMBER(){
       final int IDaction = 8;
        return new InstantAction(() ->{//todo okey
            ID = IDaction;
            bracoGarraSuperiorState =BracoGarraSuperiorStates.OUTTAKE_CHAMBER;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
   }

    public Action goToReadyHang(){
        final int IDaction = 9;
        return new InstantAction(()->{
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.READY_TO_HANG;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
            lastStateBracoSuperior = bracoGarraSuperiorState;
        });
    }
    public Action goToHang(){
        final int IDaction = 10;
        return new InstantAction(()->{
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.HANG;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
            lastStateBracoSuperior = bracoGarraSuperiorState;
        });
    }
   public void upSetPoint(double increase) { // todo: corrigido!
       double position = Range.clip(bracoGarraSuperiorServo.getPosition() + (increase * 0.005),0,1);
       bracoGarraSuperiorServo.setPosition(position);
    }
   public void downSetPoint(double decrease) { // todo: corrigido!
        double position = Range.clip(bracoGarraSuperiorServo.getPosition() + (decrease * 0.005),0,1);
        bracoGarraSuperiorServo.setPosition(position);
    }



    public void monitor(Telemetry telemetry) {
        if (monitor) {
            telemetry.addLine("==============================");
            telemetry.addLine("  TELEMETRIA DO BRAÇO DA GARRA");
            telemetry.addLine("===============================");
            telemetry.addData("BRACO PWM status: ",bracoGarraSuperiorServo.getController().getPwmStatus());
            telemetry.addData("BRACO position: ",bracoGarraSuperiorServo.getPosition());
            telemetry.addData("BRACO Estado Atual", bracoGarraSuperiorState);

            //telemetry.addData("",);

        }
    }


}
