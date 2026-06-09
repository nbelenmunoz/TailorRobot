'Settaggio velocità
Servo On
Ovrd 100
Accel 100,100
Spd 30
Cnt 0
''NUMERO PUNTI TRAIETTORIA, loop e comunicazione (si aggiorna lui)
Def Inte np,nbuff,M1,M2,M3
''Position and Speed Buff, must be >= np set on MATLAB
Dim PP(20)
Dim MVEL(20)
''HOME
PHome=(+230.00,+80.00,+290.00,-180.00,+0.00,-90.00)(6,0)
Mvs PHome
Ovrd 100
'' OPEN MATLAB COMUNICATION
*rpt
Open "COM7:" As #7
If M_Open(7) <> 1 Then *rpt
Dly 0.1
'Ricevo numero punti e primi punti
Input #7,np%
Input #7,nbuff%
For M1%=1 To nbuff% Step 1
    Input #7,PP(M1%)
    Input #7,MVEL(M1%)
Next M1%
'Aspetto OK e mi muovo al primo punto
Input #7,M2%
If M2%=0 Then
    Spd MVEL(1)
    Mvs PP(1)           ' Movimiento lineal
EndIf
Print #7,"PRONTO"
Input #7,M2%
Print #7,C_Time,P_Fbc,M_RSpd
'Inizio PROCESSO
If M2%=1 Then
    For M1%=2 To np% Step 1
        If M1%<=np%-nbuff% Then
            Input #7,PP(M2%)
            Input #7,MVEL(M2%)
        EndIf
        If M2%=nbuff% Then
            M2%=1
        Else
            M2%=M2%+1
        EndIf
        If np%-nbuff%<=2 And M1%=np% Then
            M2%=nbuff%
        EndIf
        Spd MVEL(M2%)     ' Velocidad lineal (mm/s)
        Mvs PP(M2%)       ' Movimiento lineal
        Print #7,C_Time,P_Fbc,M_RSpd
        Input #7,M3%
    Next M1%
EndIf
'Torno in home (velocidad aumentada a 100 mm/s)
Input #7,M2%
If M2%=1 Then
    Print #7,"PARTO"
    Dly 1
    Spd 100
    Mvs PHome
EndIf
Close #7
Dly 0.5
Servo Off
Hlt
