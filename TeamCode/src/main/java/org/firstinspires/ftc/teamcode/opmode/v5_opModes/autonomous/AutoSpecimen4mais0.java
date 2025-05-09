package org.firstinspires.ftc.teamcode.opmode.v5_opModes.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.V5;
import org.firstinspires.ftc.teamcode.common.roadrunner.MecanumDrive;


@Config
@Autonomous(name = "AutonomoSpecimen4+0 estavel ", group = "Autonomous")
public class AutoSpecimen4mais0 extends LinearOpMode {
    //AutoSpecimen5mais0SungandoBASE
    V5 robot;
    Action push;
    public static int target = 0,
            XGoToPush1 = 36, YGoToPush1 = -30, HGoToPush1 = -90,

    XGoToPush2 = 47, YGoToPush2 = -5, HGoToPush2 = -90,

    XPush1 = 0, YPush1 = 0, HPush1 = 0 ,
            XPush2 = 0, YPush2 = -58, HPush2 = 0 ,

    XPush3 = 54, YPush3 = -5, HPush3 = -90,

    XReadyToCollect = 43 , YReadyToCollect= -40, HReadyToCollect = 0 ,

    XCollect1 = 43 , YCollect1 = -67 , HCollect1,

    YPush3FINAL = -57,

    Xdeposit2 = -14,Ydeposit2 =-50,
            XdepositFINAL =0,YdepositFINAL =50

    ;
    public static double delayFecharGarraIntake = 1.65, velIntake = 35, posIntake = -65;

    @Override
    public void runOpMode()  {

        //todo antiga: Pose2d initialPose = new Pose2d(8.5, -61, Math.toRadians(-90));
        Pose2d initialPose = new Pose2d(8, -61, Math.toRadians(-90));

        robot = new V5(hardwareMap,telemetry);
        MecanumDrive.PARAMS.maxProfileAccel = 80;
        MecanumDrive.PARAMS.minProfileAccel = -80;
        MecanumDrive.PARAMS.maxWheelVel  = 80;

        robot.md.pose = initialPose;
        Actions.runBlocking(
                new SequentialAction(
                        //robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        new InstantAction(() -> robot.outtakeIntakeSuperior.braco.bracoGarraSuperiorServo.setPosition(0.209)),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.garraSuperior.goToInitialSpecimen()
                        //robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
                        //robot.intakeInferior.horizontalInferior.goToRetracted()

                )
        );
        push = pushSamples();
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        robot.intakeInferior.linearHorizontalMotor.turnOnHold(),
                        new InstantAction(() -> {robot.intakeInferior.linearHorizontalMotor.changeTarget(target);robot.intakeInferior.linearHorizontalMotor.PID();} ),
                        deposit(),
                        new ParallelAction(
                                new SequentialAction(
                                        //robot.outtakeIntakeSuperior.garraSuperior.abrirGarra(),
                                        robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                        positionIntake()
                                ),
                                push
                        ),
                        //firstIntake(),
                        collect(),
                        deposit2(),
                        intake(-6,-30),
                        deposit2(),
                        intake(-6,-30),
                        deposit2(),
                        irParaCasa()

                )
        );

    }
    public Action Movecomplept(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        //todo move deposito
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))
                        //chegar no sample 2 e empurrar
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(38, -35, Math.toRadians(90)), Math.toRadians(90))

                        .splineToLinearHeading(new Pose2d(38, -5, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(48, -5, Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(48, -50, Math.toRadians(90)), Math.toRadians(-90))

                        .splineToLinearHeading(new Pose2d(48, -10, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(56, -10, Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(56, -50, Math.toRadians(90)), Math.toRadians(-90))

                        .splineToLinearHeading(new Pose2d(52, -10, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(62, -10, Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(Math.toRadians(-90))
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(62, -50, Math.toRadians(90)), Math.toRadians(-90))
                        .setTangent(-135)
                        .splineToLinearHeading(new Pose2d(58, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))

                        //todo intake
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))

                        //todo intake
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo deposito
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))

                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(38, -50, Math.toRadians(-35)), Math.toRadians(-45))
                        .build()
        );
    }
    public Action goToDeposit(){
        return robot.md.actionBuilder(robot.md.pose)
                //todo: colocar primeiro specimen
                .strafeTo(new Vector2d(-6,-22))
                //.splineToConstantHeading(new Vector2d(-6, -26), Math.toRadians(90))
                //todo: Go to empurrar sample 1
                .build();

    }
    public Action pushSamples() {
        return
                robot.md.actionBuilder(robot.md.pose)
                        //todo ir para empurrar o segundo sample
                        .setTangent(Math.toRadians(-45))
                        //.splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(-90)), Math.toRadians(90))
                        //.splineToSplineHeading(new Pose2d(44, -5, Math.toRadians(-90)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(XGoToPush1, YGoToPush1, Math.toRadians(HGoToPush1)), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(XGoToPush2, YGoToPush2, Math.toRadians(HGoToPush2)), Math.toRadians(0))

                        //todo empurrar sample 2
                        .setTangent(Math.toRadians(-90))
                        //.lineToY(-58)
                        .lineToY(YPush2)

                        //todo empurrar sample 3
                        .setTangent(Math.toRadians(90))
                        //.splineToLinearHeading(new Pose2d(56, -5, Math.toRadians(-90)), Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(XPush3, YPush3, Math.toRadians(HPush3)), Math.toRadians(-90))
                        .lineToY(YPush3FINAL)

                        //todo coletar sample 1
                        .setTangent(Math.toRadians(90))
                        //.strafeToConstantHeading(new Vector2d(47, -63))
                        //.splineToSplineHeading(new Pose2d(43, -54, Math.toRadians(-90)), Math.toRadians(-90))

                        //.splineToConstantHeading(new Vector2d(43, -40), Math.toRadians(-90))
                        //.splineToSplineHeading(new Pose2d(43, -65, Math.toRadians(-90)), Math.toRadians(-90))

                        .splineToConstantHeading(new Vector2d(XReadyToCollect, YReadyToCollect), Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(XCollect1, YCollect1, Math.toRadians(-90)), Math.toRadians(-90))

                        //todo antiga: .splineToLinearHeading(new Pose2d(55, -45, Math.toRadians(-90)), Math.toRadians(-85))
                        .waitSeconds(0.5)
                        //.lineToY(-66)

                        .build();


    }
    public Action goToDeposit2() {
        return robot.md.actionBuilder(new Pose2d(51, -63, Math.toRadians(-90)))
                //todo: colocar primeiro specimen
                //.strafeTo(new Vector2d(-6,-22))
                .strafeTo(new Vector2d(Xdeposit2,-17))//(-12)
                //.setTangent(Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(Xdeposit2,-35,Math.toRadians(-90)),Math.toRadians(180))
                //.splineToSplineHeading(new Pose2d(XdepositFINAL,-17,Math.toRadians(-90)),Math.toRadians(-90))
                //.splineToSplineHeading(new Pose2d(EIXOX,-27,Math.toRadians(-90)
                .build();
    }
    public Action goToDeposit3() {
        return robot.md.actionBuilder(robot.md.pose)
                //todo: colocar primeiro specimen
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(0, -30.5), Math.toRadians(90))
                .build();
    }
    public Action firstIntake(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        //.strafeTo(new Vector2d(38 ,-35))
                        .lineToY(-63,  new TranslationalVelConstraint(30.0))
                        .build(),

                collect()

        );
    }
    public Action goToPark() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .strafeTo(new Vector2d(-5,-40)/* ,null, new ProfileAccelConstraint(-200, 200)*/)
                        .build()
        );

    }
    public Action deposit() {
        return new SequentialAction(
                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(1150),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeSpecimen(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit()
                        )
                ),

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.25).build()
                //robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );

    }
    public Action  deposit2() {
        return new SequentialAction(
                new ParallelAction(
                        robot.outtakeIntakeSuperior.braco.goToOuttakeCHAMBER(),
                        robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(1150),
                        robot.outtakeIntakeSuperior.garraSuperior.goToOuttakeSpecimen(),
                        new SequentialAction(
                                //robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit2()
                        )
                ),

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.25).build()
               // robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );

    }


    public Action intake(double x , double y){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                new ParallelAction(
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                positionIntake(),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(delayFecharGarraIntake).build(),
                                collect()
                        ),
                        robot.md.actionBuilder(new Pose2d(x, y, Math.toRadians(-90)))
                                //.strafeTo(new Vector2d(38 ,-35))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(51, -55, Math.toRadians(-90)), Math.toRadians(-90))
                                //.waitSeconds(0.4)
                                //.lineToY(posIntake,null, new ProfileAccelConstraint(-velIntake, velIntake))
                                .build()
                ),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.30).build()


        );
    }

    public  Action positionIntake() {
        return new ParallelAction(
                //robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()                                                                                                                                                                                                           ,
                robot.outtakeIntakeSuperior.braco.goToIntakeCHAMBER(),
                robot.outtakeIntakeSuperior.garraSuperior.goToIntakeSpecimen(),
                robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(-100)


        );
    }
    public Action collect() {
        return new SequentialAction(
                //obot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.45).build()
        );
    }
    public Action collect2() {
        return new SequentialAction(
                //new InstantAction(()->{robot.outtakeIntakeSuperior.garraSuperior.servoAberturaDaGarra.setPosition(2);}),
                robot.md.actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.8).build()
                //robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build()
        );
    }

    public Action sample1() {
        return new SequentialAction(
                new ParallelAction(
                        deposit(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                                goToDeposit()
                        )

                ),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.6).build()
                //robot.outtakeIntakeSuperior.garraSuperior.abrirGarra()
        );
    }
    public Action sample2() {
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build(),
                //intake(),
                deposit()
        );
    }
    public Action sample3() {
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                //intake(),
                deposit()
        );
    }
    public Action sample4(){
        return new SequentialAction(

                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                //intake(),
                deposit()

        );
    }

    public Action irParaCasa(){
        return new SequentialAction(
                new ParallelAction(
                        goToPark(),
                        new SequentialAction(
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.25).build(),
                                robot.outtakeIntakeSuperior.braco.goToInital(),
                                robot.outtakeIntakeSuperior.linearVertical.ElevadorGoTo(0)
                        )

                )


        );
    }

}