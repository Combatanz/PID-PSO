AutoShop            2.4  5.03                                                                                                           ��  CLVTItem              �             �             �             ��//������Ա��Jason
//�����ص㣺Tsinghua University
//����ʱ�䣺2024.9.25
//�������ݣ�PID���Ƴ���(����kalman�˲�)


//������·��ϵͳ��Ҫ�ɼ������б���
VAR 
PRESSURE1    AT%ID200  : REAL ;
PRESSURE2    AT%ID210  : REAL ;
PRESSURE3    AT%ID220  : REAL ;
PRESSURE4    AT%ID230  : REAL ;
PRESSURE5    AT%ID240  : REAL ;
PRESSURE6    AT%ID250  : REAL ;
TEMPRETURE   AT%ID300  : REAL ;
FLOWREAD1    at%ID320  : REAL ;
PROPORVALUE  AT%ID340  : REAL ;
COUNTERVALUE AT%ID360  : REAL ;
PROPORWRITE  AT%QD2000 : REAL ;    //REAL ������: 32λ�������������ȸ���������ֵ��Χ: ��Լ 3.4E?38 �� 3.4E+38������ 7 λ��Ч���֡���;: ���ڴ洢����С������ֵ�����ٶȡ�ѹ���������ȡ�
COUNTERWITE  AT%QD3000 : REAL ;    //DWORD ����: 32λ�޷���������ֵ��Χ: 0 �� 4,294,967,295��
TEST1        AT%I*     : REAL ;
i            AT%I*     : INT  ;   //��INT���� �����͵� ����  -32768-32768��
p            AT%I*     : ARRAY[1..6] OF REAL;
a            AT%I*     : ARRAY[1..6] OF REAL := [PRESSURE1,PRESSURE2,PRESSURE3,PRESSURE4,PRESSURE5,PRESSURE6];
END_VAR




//��ѹ������������ֵ���ӻ�����
for  i := 1 to 6 by 1 DO
	p[i] := a[i] / 39.6 ;
END_FOR
p[1];p[2];p[3];p[4];p[5];p[6];






//����PI�������ʹ�÷���
VAR  
    fbpid    :  FB_PID  ;            //����  ���� �Լ��Ĵ����ĺ�����                          
	Por          AT%I*     : REAL  ;    //ʹ�ú������  ���� �Լ�  ���
    Inter        AT%I*     : REAL  ;
    ExpecValue   AT%I*     : REAL  ;
    Bias1        AT%I*     : REAL  ;
    RealityValue AT%I*     : REAL  ;
    OutputPress  AT%I*     : REAL  ;
    Error1       AT%I*     : REAL  ;
END_VAR  
//������  ������ �ĸ�ֵ
RealityValue := p[4] ;
ExpecValue   := 200.0;               //��λ�� 200kpa  �Ѿ�����ת����
Por          := 1    ;
Inter        := 0.1  ;
Bias1        := 10   ;
fbpid.IN1    :=Por   ;
fbpid.IN2    :=Inter ;
fbpid.IN11   :=Bias1 ;               //����  ������  input  ��ֵ
fbpid(IN5 := RealityValue,IN4 := ExpecValue);   //����  ����  ��������� ����˿�
OutputPress  :=fbpid.OUT3;           //ʹ��  ������  ��������Ӷ�  ���ֵ
Error1       :=fbpid.OUT6;

//��Ҫ��  OutputPress�������  ת�� Ϊ  �������Ŀ��ȣ�Ҳ���ǿ��ȸ����٣��ܴﵽOutputPress����ֵ

























//(* ����PID�������Ĳ����ͱ��� *)
//VAR
//    Kp                 AT%I*: REAL := 1.0;          (* ����ϵ�� *)
//    Ki                 AT%I*: REAL := 0.5;          (* ����ϵ�� *)
//    Kd                 AT%I*: REAL := 0.1;          (* ΢��ϵ�� *)
//    Setpoint           AT%I*: REAL := 100.0;  (* Ŀ���趨ֵ *)
//    ProcessValue       AT%I*: REAL;       (* ��ǰϵͳʵ��ֵ�������¶� *)
//    Error              AT%I*: REAL;              (* �����趨ֵ��ʵ��ֵ�Ĳ� *)
//    IntegralSum        AT%I*: REAL := 0.0; (* ���ֺͱ��� *)
//    Bias               AT%I*: REAL := 10.0;       (* ƫ����������Ϊ10�Է�ֹ���Ϊ0 *)
//    Output             AT%I*: REAL;             (* PID����������� *)
//    DeltaTime          AT%I*: REAL := 0.1;   (* ÿ��ѭ����ʱ����������Ϊ0.1��     Ҳ������Sample Time       *)   
//    IntegralLimit      AT%I*: REAL := 100.0; (* �������ƣ���ֹ���ֱ��� *)
//	JogDirection       AT%I*:=BOOL;    //ת������ѡ��
//	OutputVariable     AT%I*:=REAL;  //Know the MC_Power's status
//	
//END_VAR
//
//(* PID�����㷨�������� *)
//Error := Setpoint - ProcessValue;    (* ���㵱ǰ��� *)
//
//(* ����������� *)
//VAR
//P_output: REAL;
//P_output := Kp * Error;
//END_VAR
//
//(* ������ֲ��ֲ����ƻ��ֺ� *)
//VAR 
//I_output: REAL;
//IntegralSum := IntegralSum + (Ki * Error * DeltaTime);   (*������������ʵ�ֵģ�һ�㲻����Ϊ0*)//������ӵ��  bias ��
//END_VAR
//
//(* ��ֹ���ֺͳ����趨�����ƣ�����Ҫ *)  
//IF IntegralSum > IntegralLimit THEN
//    IntegralSum := IntegralLimit;
//ELSIF IntegralSum < -IntegralLimit THEN
//    IntegralSum := -IntegralLimit;
//END_IF;
//(* ����΢�ֲ��� *)
//VAR 
//D_output: REAL;
//D_output := Kd * (Error / DeltaTime);
//END_VAR
//(* �����ܵ�PID���������Biasƫ���� *)
//Output := P_output + I_output + D_output + Bias;     //PID�����㷨ʹ��ʱ�򣬱��뱣��  ԣ�ȣ�Ҳ����  Bias  �ĺ��塣
//
//(* ���ֵ�����ƣ�ȷ������ں�����Χ�ڣ�����0��100֮�� *)
//IF Output > 100.0 THEN
//    Output := 100.0;
//ELSIF Output < 0.0 THEN
//    Output := 0.0;
//END_IF;
//
//(* ����ProcessValue������ϵͳģ�ͻ򴫸����ķ��� *)
//ProcessValue := UpdateProcessValue(Output);        (* ���������ģ�����ģ�� *)

//MC_Power(Enable := ???,
//         Axis := Axis_0,
//         Status => ,
//         Busy => ,
//         Error => ,
//         ErrorID => );    //����ṹ����  ��  ������ �ĸ�ֵ��䣬��������Ϊ����һ������������������������Լ����������
// MC_ReadStatus(Enable := ???,
//              Axis := Axis_0,
//              Valid => ,
//              Busy => ,
//              Disabled => ,
//              ErrorStop => ,
//              Stopping => ,
//              Standstill => ,
//              DiscreteMotion => ,
//              ContinuousMotion => ,
//              SynchronizedMotion => ,
//              Homing => ,
//              ConstantVelocity => ,
//              Accelerating => ,
//              Decelerating => ,
//              Error => ,
//              ErrorID => );               //  ��=> Ϊ��������������ң� = �����������
//
//OutputVariable:= MC_Power.Status;        //reading the valus about the MC_power's status  ������ֵ
//
//IF  JogDirection THEN
//	MC_Jog.JogForward:=1;               //ѡ��ģʽ  Mode Chosen    �����Ժ������ڲ������ĸ�ֵ����䣩��
//ELSE 
//	MC_Jog.JogBJogBackward :=1;        //����  ��������  ��    �����籣�֣�
//	END_IF                           
//	
//
//    