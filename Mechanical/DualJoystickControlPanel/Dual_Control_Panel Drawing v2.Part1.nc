( Made using CamBam - http://www.cambam.co.uk )
( Dual_Control_Panel Drawing v2 5/7/2023 9:27:55 AM )
( T1 : 0.125 )
( T2 : 0.125 )
( T3 : 0.125 )
( T4 : 0.1875 )
G20 G90 G64 G40
G0 Z0.125
( T1 : 0.125 )
T1 M6
( SmallSpotDrill )
G17
M3 S1000
G0 X0.5005 Y1.2175
G98
G81 X0.5005 Y1.2175 Z-0.075 R0.125 F3.0
G81 X0.9605 Y0.6075 Z-0.075
G81 Y-0.6725 Z-0.075
G81 X0.5005 Y-1.2825 Z-0.075
G81 X2.2405 Y-0.6725 Z-0.075
G81 X3.0005 Y-1.4625 Z-0.075
G81 X3.7605 Y-0.6725 Z-0.075
G81 Y0.6075 Z-0.075
G81 X3.0005 Y0.7175 Z-0.075
G81 Y1.3975 Z-0.075
G81 X2.2405 Y0.6075 Z-0.075
G81 X5.0405 Z-0.075
G81 X5.5005 Y1.2175 Z-0.075
G81 X5.0405 Y-0.6725 Z-0.075
G81 X5.5005 Y-1.2825 Z-0.075
G80
( MountingHoles )
G0 Z0.125
( T2 : 0.125 )
T2 M6
M3 S1000
G0 X0.5005 Y1.2175
G98
G81 X0.5005 Y1.2175 Z-0.25 R0.125 F3.0
G81 Y-1.2825 Z-0.25
G81 X3.0005 Y-1.4625 Z-0.25
G81 Y0.7175 Z-0.25
G81 Y1.3975 Z-0.25
G81 X5.5005 Y1.2175 Z-0.25
G81 Y-1.2825 Z-0.25
G80
( JoystickHoles )
G0 Z0.125
( T3 : 0.125 )
T3 M6
M3 S1000
G0 X0.9605 Y0.6075
G98
G81 X0.9605 Y0.6075 Z-0.25 R0.125 F3.0
G81 Y-0.6725 Z-0.25
G81 X2.2405 Z-0.25
G81 Y0.6075 Z-0.25
G81 X3.7605 Z-0.25
G81 Y-0.6725 Z-0.25
G81 X5.0405 Z-0.25
G81 Y0.6075 Z-0.25
G80
( JoystickHole_Rough )
G0 Z0.125
( T4 : 0.1875 )
T4 M6
M3 S1000
G0 X2.3017 Y-0.0325
G0 Z0.0625
G1 F3.0 Z-0.02
G2 F10.0 X1.2499 Y-0.6398 I-0.7013 J0.0
G2 Y0.5748 I0.3506 J0.6073
G2 X2.3017 Y-0.0325 I0.3506 J-0.6073
G1 F3.0 Z-0.04
G2 F10.0 X1.2499 Y-0.6398 I-0.7013 J0.0
G2 Y0.5748 I0.3506 J0.6073
G2 X2.3017 Y-0.0325 I0.3506 J-0.6073
G1 F3.0 Z-0.06
G2 F10.0 X1.2499 Y-0.6398 I-0.7013 J0.0
G2 Y0.5748 I0.3506 J0.6073
G2 X2.3017 Y-0.0325 I0.3506 J-0.6073
G1 F3.0 Z-0.08
G2 F10.0 X1.2499 Y-0.6398 I-0.7013 J0.0
G2 Y0.5748 I0.3506 J0.6073
G2 X2.3017 Y-0.0325 I0.3506 J-0.6073
G1 F3.0 Z-0.1
G2 F10.0 X1.2499 Y-0.6398 I-0.7013 J0.0
G2 Y0.5748 I0.3506 J0.6073
G2 X2.3017 Y-0.0325 I0.3506 J-0.6073
G1 F3.0 Z-0.12
G2 F10.0 X1.2499 Y-0.6398 I-0.7013 J0.0
G2 Y0.5748 I0.3506 J0.6073
G2 X2.3017 Y-0.0325 I0.3506 J-0.6073
G1 F3.0 Z-0.14
G2 F10.0 X1.2499 Y-0.6398 I-0.7013 J0.0
G2 Y0.5748 I0.3506 J0.6073
G2 X2.3017 Y-0.0325 I0.3506 J-0.6073
G0 Z0.125
G0 X3.6992
G0 Z0.0625
G1 F3.0 Z-0.02
G2 F10.0 X4.0499 Y0.5748 I0.7013 J0.0
G2 X5.1017 Y-0.0325 I0.3506 J-0.6073
G2 X4.0499 Y-0.6398 I-0.7013 J0.0
G2 X3.6992 Y-0.0325 I0.3506 J0.6073
G1 F3.0 Z-0.04
G2 F10.0 X4.0499 Y0.5748 I0.7013 J0.0
G2 X5.1017 Y-0.0325 I0.3506 J-0.6073
G2 X4.0499 Y-0.6398 I-0.7013 J0.0
G2 X3.6992 Y-0.0325 I0.3506 J0.6073
G1 F3.0 Z-0.06
G2 F10.0 X4.0499 Y0.5748 I0.7013 J0.0
G2 X5.1017 Y-0.0325 I0.3506 J-0.6073
G2 X4.0499 Y-0.6398 I-0.7013 J0.0
G2 X3.6992 Y-0.0325 I0.3506 J0.6073
G1 F3.0 Z-0.08
G2 F10.0 X4.0499 Y0.5748 I0.7013 J0.0
G2 X5.1017 Y-0.0325 I0.3506 J-0.6073
G2 X4.0499 Y-0.6398 I-0.7013 J0.0
G2 X3.6992 Y-0.0325 I0.3506 J0.6073
G1 F3.0 Z-0.1
G2 F10.0 X4.0499 Y0.5748 I0.7013 J0.0
G2 X5.1017 Y-0.0325 I0.3506 J-0.6073
G2 X4.0499 Y-0.6398 I-0.7013 J0.0
G2 X3.6992 Y-0.0325 I0.3506 J0.6073
G1 F3.0 Z-0.12
G2 F10.0 X4.0499 Y0.5748 I0.7013 J0.0
G2 X5.1017 Y-0.0325 I0.3506 J-0.6073
G2 X4.0499 Y-0.6398 I-0.7013 J0.0
G2 X3.6992 Y-0.0325 I0.3506 J0.6073
G1 F3.0 Z-0.14
G2 F10.0 X4.0499 Y0.5748 I0.7013 J0.0
G2 X5.1017 Y-0.0325 I0.3506 J-0.6073
G2 X4.0499 Y-0.6398 I-0.7013 J0.0
G2 X3.6992 Y-0.0325 I0.3506 J0.6073
( JoystickHole_Finish )
S1000
G0 Z0.125
G0 X2.3117
G0 Z0.0625
G1 F3.0 Z-0.02
G2 F10.0 X1.2449 Y-0.6485 I-0.7113 J0.0
G2 Y0.5834 I0.3556 J0.616
G2 X2.3117 Y-0.0325 I0.3556 J-0.616
G1 F3.0 Z-0.04
G2 F10.0 X1.2449 Y-0.6485 I-0.7113 J0.0
G2 Y0.5834 I0.3556 J0.616
G2 X2.3117 Y-0.0325 I0.3556 J-0.616
G1 F3.0 Z-0.06
G2 F10.0 X1.2449 Y-0.6485 I-0.7113 J0.0
G2 Y0.5834 I0.3556 J0.616
G2 X2.3117 Y-0.0325 I0.3556 J-0.616
G1 F3.0 Z-0.08
G2 F10.0 X1.2449 Y-0.6485 I-0.7113 J0.0
G2 Y0.5834 I0.3556 J0.616
G2 X2.3117 Y-0.0325 I0.3556 J-0.616
G1 F3.0 Z-0.1
G2 F10.0 X1.2449 Y-0.6485 I-0.7113 J0.0
G2 Y0.5834 I0.3556 J0.616
G2 X2.3117 Y-0.0325 I0.3556 J-0.616
G1 F3.0 Z-0.12
G2 F10.0 X1.2449 Y-0.6485 I-0.7113 J0.0
G2 Y0.5834 I0.3556 J0.616
G2 X2.3117 Y-0.0325 I0.3556 J-0.616
G1 F3.0 Z-0.14
G2 F10.0 X1.2449 Y-0.6485 I-0.7113 J0.0
G2 Y0.5834 I0.3556 J0.616
G2 X2.3117 Y-0.0325 I0.3556 J-0.616
G0 Z0.125
G0 X3.6892
G0 Z0.0625
G1 F3.0 Z-0.02
G2 F10.0 X4.0449 Y0.5834 I0.7113 J0.0
G2 X5.1117 Y-0.0325 I0.3556 J-0.616
G2 X4.0449 Y-0.6485 I-0.7113 J0.0
G2 X3.6892 Y-0.0325 I0.3556 J0.616
G1 F3.0 Z-0.04
G2 F10.0 X4.0449 Y0.5834 I0.7113 J0.0
G2 X5.1117 Y-0.0325 I0.3556 J-0.616
G2 X4.0449 Y-0.6485 I-0.7113 J0.0
G2 X3.6892 Y-0.0325 I0.3556 J0.616
G1 F3.0 Z-0.06
G2 F10.0 X4.0449 Y0.5834 I0.7113 J0.0
G2 X5.1117 Y-0.0325 I0.3556 J-0.616
G2 X4.0449 Y-0.6485 I-0.7113 J0.0
G2 X3.6892 Y-0.0325 I0.3556 J0.616
G1 F3.0 Z-0.08
G2 F10.0 X4.0449 Y0.5834 I0.7113 J0.0
G2 X5.1117 Y-0.0325 I0.3556 J-0.616
G2 X4.0449 Y-0.6485 I-0.7113 J0.0
G2 X3.6892 Y-0.0325 I0.3556 J0.616
G1 F3.0 Z-0.1
G2 F10.0 X4.0449 Y0.5834 I0.7113 J0.0
G2 X5.1117 Y-0.0325 I0.3556 J-0.616
G2 X4.0449 Y-0.6485 I-0.7113 J0.0
G2 X3.6892 Y-0.0325 I0.3556 J0.616
G1 F3.0 Z-0.12
G2 F10.0 X4.0449 Y0.5834 I0.7113 J0.0
G2 X5.1117 Y-0.0325 I0.3556 J-0.616
G2 X4.0449 Y-0.6485 I-0.7113 J0.0
G2 X3.6892 Y-0.0325 I0.3556 J0.616
G1 F3.0 Z-0.14
G2 F10.0 X4.0449 Y0.5834 I0.7113 J0.0
G2 X5.1117 Y-0.0325 I0.3556 J-0.616
G2 X4.0449 Y-0.6485 I-0.7113 J0.0
G2 X3.6892 Y-0.0325 I0.3556 J0.616
( OutsideProfile_Rough )
S1000
G0 Z0.125
G0 X6.1042 Y-1.2825
G0 Z0.0625
G1 F3.0 Z-0.02
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8212 I-0.6038 J0.0
G1 X0.5005
G3 X-0.1033 Y1.2175 I0.0 J-0.6038
G1 Y-1.2825
G3 X0.5005 Y-1.8863 I0.6038 J0.0
G1 X5.5005
G3 X6.1042 Y-1.2825 I0.0 J0.6038
G1 F3.0 Z-0.04
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8212 I-0.6038 J0.0
G1 X0.5005
G3 X-0.1033 Y1.2175 I0.0 J-0.6038
G1 Y-1.2825
G3 X0.5005 Y-1.8863 I0.6038 J0.0
G1 X5.5005
G3 X6.1042 Y-1.2825 I0.0 J0.6038
G1 F3.0 Z-0.06
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8212 I-0.6038 J0.0
G1 X0.5005
G3 X-0.1033 Y1.2175 I0.0 J-0.6038
G1 Y-1.2825
G3 X0.5005 Y-1.8863 I0.6038 J0.0
G1 X5.5005
G3 X6.1042 Y-1.2825 I0.0 J0.6038
G1 F3.0 Z-0.08
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8212 I-0.6038 J0.0
G1 X0.5005
G3 X-0.1033 Y1.2175 I0.0 J-0.6038
G1 Y-1.2825
G3 X0.5005 Y-1.8863 I0.6038 J0.0
G1 X5.5005
G3 X6.1042 Y-1.2825 I0.0 J0.6038
G1 F3.0 Z-0.1
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8212 I-0.6038 J0.0
G1 X0.5005
G3 X-0.1033 Y1.2175 I0.0 J-0.6038
G1 Y-1.2825
G3 X0.5005 Y-1.8863 I0.6038 J0.0
G1 X5.5005
G3 X6.1042 Y-1.2825 I0.0 J0.6038
G1 F3.0 Z-0.12
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8212 I-0.6038 J0.0
G1 X0.5005
G3 X-0.1033 Y1.2175 I0.0 J-0.6038
G1 Y-1.2825
G3 X0.5005 Y-1.8863 I0.6038 J0.0
G1 X5.5005
G3 X6.1042 Y-1.2825 I0.0 J0.6038
G1 F3.0 Z-0.14
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8212 I-0.6038 J0.0
G1 X0.5005
G3 X-0.1033 Y1.2175 I0.0 J-0.6038
G1 Y-1.2825
G3 X0.5005 Y-1.8863 I0.6038 J0.0
G1 X5.5005
G3 X6.1042 Y-1.2825 I0.0 J0.6038
( OutsideProfile_Finish )
S1000
G0 Z-0.02
G1 F3.0 X6.0992
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8162 I-0.5988 J0.0
G1 X0.5005
G3 X-0.0983 Y1.2175 I0.0 J-0.5988
G1 Y-1.2825
G3 X0.5005 Y-1.8813 I0.5988 J0.0
G1 X5.5005
G3 X6.0992 Y-1.2825 I0.0 J0.5988
G1 F3.0 Z-0.04
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8162 I-0.5988 J0.0
G1 X0.5005
G3 X-0.0983 Y1.2175 I0.0 J-0.5988
G1 Y-1.2825
G3 X0.5005 Y-1.8813 I0.5988 J0.0
G1 X5.5005
G3 X6.0992 Y-1.2825 I0.0 J0.5988
G1 F3.0 Z-0.06
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8162 I-0.5988 J0.0
G1 X0.5005
G3 X-0.0983 Y1.2175 I0.0 J-0.5988
G1 Y-1.2825
G3 X0.5005 Y-1.8813 I0.5988 J0.0
G1 X5.5005
G3 X6.0992 Y-1.2825 I0.0 J0.5988
G1 F3.0 Z-0.08
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8162 I-0.5988 J0.0
G1 X0.5005
G3 X-0.0983 Y1.2175 I0.0 J-0.5988
G1 Y-1.2825
G3 X0.5005 Y-1.8813 I0.5988 J0.0
G1 X5.5005
G3 X6.0992 Y-1.2825 I0.0 J0.5988
G1 F3.0 Z-0.1
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8162 I-0.5988 J0.0
G1 X0.5005
G3 X-0.0983 Y1.2175 I0.0 J-0.5988
G1 Y-1.2825
G3 X0.5005 Y-1.8813 I0.5988 J0.0
G1 X5.5005
G3 X6.0992 Y-1.2825 I0.0 J0.5988
G1 F3.0 Z-0.12
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8162 I-0.5988 J0.0
G1 X0.5005
G3 X-0.0983 Y1.2175 I0.0 J-0.5988
G1 Y-1.2825
G3 X0.5005 Y-1.8813 I0.5988 J0.0
G1 X5.5005
G3 X6.0992 Y-1.2825 I0.0 J0.5988
G1 F3.0 Z-0.14
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8162 I-0.5988 J0.0
G1 X0.5005
G3 X-0.0983 Y1.2175 I0.0 J-0.5988
G1 Y-1.2825
G3 X0.5005 Y-1.8813 I0.5988 J0.0
G1 X5.5005
G3 X6.0992 Y-1.2825 I0.0 J0.5988
( OutsideProfile_Finish2 )
S1000
G0 Z-0.02
G1 F3.0 X6.0942
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8112 I-0.5938 J0.0
G1 X0.5005
G3 X-0.0933 Y1.2175 I0.0 J-0.5938
G1 Y-1.2825
G3 X0.5005 Y-1.8763 I0.5938 J0.0
G1 X5.5005
G3 X6.0942 Y-1.2825 I0.0 J0.5938
G1 F3.0 Z-0.04
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8112 I-0.5938 J0.0
G1 X0.5005
G3 X-0.0933 Y1.2175 I0.0 J-0.5938
G1 Y-1.2825
G3 X0.5005 Y-1.8763 I0.5938 J0.0
G1 X5.5005
G3 X6.0942 Y-1.2825 I0.0 J0.5938
G1 F3.0 Z-0.06
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8112 I-0.5938 J0.0
G1 X0.5005
G3 X-0.0933 Y1.2175 I0.0 J-0.5938
G1 Y-1.2825
G3 X0.5005 Y-1.8763 I0.5938 J0.0
G1 X5.5005
G3 X6.0942 Y-1.2825 I0.0 J0.5938
G1 F3.0 Z-0.08
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8112 I-0.5938 J0.0
G1 X0.5005
G3 X-0.0933 Y1.2175 I0.0 J-0.5938
G1 Y-1.2825
G3 X0.5005 Y-1.8763 I0.5938 J0.0
G1 X5.5005
G3 X6.0942 Y-1.2825 I0.0 J0.5938
G1 F3.0 Z-0.1
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8112 I-0.5938 J0.0
G1 X0.5005
G3 X-0.0933 Y1.2175 I0.0 J-0.5938
G1 Y-1.2825
G3 X0.5005 Y-1.8763 I0.5938 J0.0
G1 X5.5005
G3 X6.0942 Y-1.2825 I0.0 J0.5938
G1 F3.0 Z-0.12
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8112 I-0.5938 J0.0
G1 X0.5005
G3 X-0.0933 Y1.2175 I0.0 J-0.5938
G1 Y-1.2825
G3 X0.5005 Y-1.8763 I0.5938 J0.0
G1 X5.5005
G3 X6.0942 Y-1.2825 I0.0 J0.5938
G1 F3.0 Z-0.14
G1 F10.0 Y1.2175
G3 X5.5005 Y1.8112 I-0.5938 J0.0
G1 X0.5005
G3 X-0.0933 Y1.2175 I0.0 J-0.5938
G1 Y-1.2825
G3 X0.5005 Y-1.8763 I0.5938 J0.0
G1 X5.5005
G3 X6.0942 Y-1.2825 I0.0 J0.5938
G0 Z0.125
M5
M30