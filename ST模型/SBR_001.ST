AutoShop            2.4  5.03                                                                                                           ��  CLVTItem              �             �             �             ��VAR
Kp:=REAL;
Ki:=REAL;

END_VAR
































//MC_Power(Enable := 16#01,
//         Axis := 0,
//         Status =>1 ,
//         Busy =>2 ,
//         Error =>16#16 ,
//         ErrorID => A);                //Ҳ����˵��һ��  Axis0��  ֻ�ǿ���  Ŀ�����  ʹ��״̬��һ��������ֻ�ǿ���һ���� ,�˶�����ֻ��MC�����.
//
//MC_ReadStatus(Enable := ???,
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
//              ErrorID => );
//
//MC_Jog(Enable := 1,    //����ת��    (MC_****)  ��������һ��  �����飬��Function Block��
//       Axis := Axis_1,
//       JogForward := 1,
//       JogBackward := 0,
//       Velocity := ???,
//       Acceleration := ???,
//       Deceleration := ,
//       CurveType := ,
//       Busy => ,
//       CommandAborted => ,             //ͨ����ST������   (=>)   ����  ���  �ĺ���
//       Error => ,
//       ErrorID => );
//MC_Jog(Enable := 1,   //����ת��
//       Axis := Axis_1,
//       JogForward := 0,
//       JogBackward := 1,
//       Velocity := ???,
//       Acceleration := ???,
//       Deceleration := ,
//       CurveType := ,
//       Busy => ,
//       CommandAborted => ,
//       Error => ,
//       ErrorID => );          //��θ���  MC_Jog  ģ�������ת��ģʽ�л���ʹ���ж���䣡����
//
//    