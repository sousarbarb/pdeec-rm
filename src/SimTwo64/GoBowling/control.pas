const
  DEBUG = true;
  NUM_JOINTS = 7;

// Global Variables
var
  iBall, iRobot, iMagnet, iB0, iB6: integer;
  l0, l1, l2, l3, lt: double;

  magnet: boolean;
  BallPos: Matrix;
  JointPos, JointVel, JointPosRef: array[0..NUM_JOINTS - 1] of double;



procedure CommunicationLazarus;
var
  i: integer;
  mess: TStringList;
  tmp: string;
begin
  mess := TStringList.create;
  try
    // S2 > Lz
    for i := 0 to NUM_JOINTS -1 do begin
      mess.add(format('%.4g',[JointPos[i]]));
    end;
    for i := 0 to NUM_JOINTS -1 do begin
      mess.add(format('%.4g',[JointVel[i]]));
    end;
    for i := 0 to 2 do begin
      mess.add(format('%.4g',[Mgetv(BallPos,i,0)]));
    end;

    WriteUDPData('127.0.0.1', 9809, mess.text);

    // Lz > S2
    tmp := ReadUDPData();
    if tmp <> '' then begin
      mess.text := tmp;
      if mess.count >= NUM_JOINTS then begin
        for i := 0 to NUM_JOINTS -1 do begin
          JointPosRef[i] := StrToFloat(mess.strings[i]);
        end;
      end;
      if mess.count >= NUM_JOINTS + 1 then begin
        if (StrToInt(mess.strings[NUM_JOINTS]) = 1) then begin
          magnet := true;
        end else begin
          magnet := false;
        end;
      end;
    end;

  finally
    mess.free;
  end;
end;

procedure VisualizeSheet;
var
  i: integer;
begin
  // - update joints state
  for i:=0 to NUM_JOINTS-1 do begin
    if (i <> 0) then begin
      SetRCValue(2,2+i, Format('%.6g', [GetAxisPosDeg(iRobot,i)]) );
      SetRCValue(3,2+i, Format('%.6g', [GetAxisPosRefDeg(iRobot,i)]) );
    end else begin
      SetRCValue(2,2+i, Format('%.6g', [GetAxisPos(iRobot,i)]) );
      SetRCValue(3,2+i, Format('%.6g', [GetAxisPosRef(iRobot,i)]) );
    end;
    SetRCValue(4,2+i, Format('%.6g', [GetAxisSpeed(iRobot,i)]) );
    SetRCValue(5,2+i, Format('%.6g', [GetAxisSpeedRef(iRobot,i)]) );
    SetRCValue(6,2+i, Format('%.6g', [GetAxisU(iRobot,i)]) );
    SetRCValue(7,2+i, Format('%.6g', [GetAxisI(iRobot,i)]) );
    SetRCValue(8,2+i, Format('%.6g', [GetAxisTorque(iRobot,i)]) );
    SetRCValue(9,2+i, Format('%d', [GetMotorControllerState(iRobot,i)]) );
    SetRCValue(10,2+i,Format('%s', [GetMotorControllerMode(iRobot,i)]) );
  end;
  // - B6
  MatrixToRangeF(13,2,GetSolidPosMat(iRobot,iB6),'%.6g');
  MatrixToRangeF(13,4,GetSolidRotMat(iRobot,iB6),'%.6g');
  MatrixToRangeF(13,8,Msub(GetSolidPosMat(iRobot,iB6),GetSolidPosMat(iRobot,iB0)),'%.6g');
  MatrixToRangeF(13,10,GetSolidRotMat(iRobot,iB6),'%.6g');
  // - Ball
  MatrixToRangeF(18,2,BallPos,'%.6g');
  // - Solenoid
  SetRCValue(17,6, Format('%.1g', [GetSensorVal(iRobot,iMagnet)]) );
  SetRCValue(18,6, Format('%d', [magnet]) );
end;

procedure Control;
var
  i: integer;
begin
  // Change ball position
  if (RCButtonPressed(17,3)) then begin
    SetSolidPosMat(iBall,0,RangeToMatrix(18,3,3,1));
  end;

  // Change solenoid state (tests wo/ communication w/ Lz)
  if (RCButtonPressed(17,7)) then begin
    magnet := true;
  end;
  if (RCButtonPressed(17,8)) then begin
    magnet := false;
  end;

  // Read Ball
  BallPos := GetSolidPosMat(iBall,0);

  // Read joint positions and velocities
  for i:=0 to NUM_JOINTS-1 do begin
    JointPos[i] := GetAxisPos(iRobot, i);
  end;
  for i:=0 to NUM_JOINTS-1 do begin
    JointVel[i] := GetAxisSpeed(iRobot, i);
  end;

  // Read UDP data + send joints state
  CommunicationLazarus;

  // Set joint reference positions
  for i:=0 to NUM_JOINTS-1 do begin
    SetAxisPosRef(iRobot, i, JointPosRef[i]);
  end;

  // Set solenoid state
  if (magnet) then begin
    SetSensorVin(iRobot,iMagnet,1);
  end else begin
    SetSensorVin(iRobot,iMagnet,0);
  end;

  // Display debug information in the sheet
  if (DEBUG) then
    VisualizeSheet;
end;


procedure Initialize;
var
  i: integer;
begin
  BallPos:= Mzeros(3,1);
  magnet := false;
  iBall  := GetRobotIndex('ball');
  iRobot := GetRobotIndex('Arm7D');
  iMagnet:= GetSensorIndex(iRobot,'grab');
  iB0    := GetSolidIndex(iRobot,'B0');
  iB6    := GetSolidIndex(iRobot,'B6');

  // Set links lengths
  l1 := 0.3;
  l2 := 0.4;
  l3 := 0.37;
  lt := 0.11;

  // Reset sheet
  SetRCValue(1,1,'JOINTS');
  SetRCValue(2,1,'p:');
  SetRCValue(3,1,'p_ref:');
  SetRCValue(4,1,'w:');
  SetRCValue(5,1,'w_ref:');
  SetRCValue(6,1,'u:');
  SetRCValue(7,1,'i:');
  SetRCValue(8,1,'t:');
  SetRCValue(9,1,'ctrl:');
  SetRCValue(10,1,'ctrl_mod:');
  SetRCValue(2,2+NUM_JOINTS,'(m/deg)');
  SetRCValue(3,2+NUM_JOINTS,'(deg)');
  SetRCValue(4,2+NUM_JOINTS,'(rad/s)');
  SetRCValue(5,2+NUM_JOINTS,'(rad/s)');
  SetRCValue(6,2+NUM_JOINTS,'(V)');
  SetRCValue(7,2+NUM_JOINTS,'(A)');
  SetRCValue(8,2+NUM_JOINTS,'(N/m)');
  SetRCValue(9,2+NUM_JOINTS,'(on/off)');
  SetRCValue(10,2+NUM_JOINTS,'');
  for i:=0 to NUM_JOINTS-1 do begin
    SetRCValue(1,2+i, Format('%d',[i]) );
  end;
  SetRCValue(12,1,'B6');
  SetRCValue(12,2,'PosW:');
  SetRCValue(13,1,'x:');
  SetRCValue(14,1,'y:');
  SetRCValue(15,1,'z:');
  SetRCValue(12,3,'RW:');
  SetRCValue(13,3,'(0):');
  SetRCValue(14,3,'(1):');
  SetRCValue(15,3,'(2):');
  SetRCValue(12,4,'(0):');
  SetRCValue(12,5,'(1):');
  SetRCValue(12,6,'(2):');
  SetRCValue(12,8,'Pos0:');
  SetRCValue(13,7,'x:');
  SetRCValue(14,7,'y:');
  SetRCValue(15,7,'z:');
  SetRCValue(12,9,'R0:');
  SetRCValue(13,9,'(0):');
  SetRCValue(14,9,'(1):');
  SetRCValue(15,9,'(2):');
  SetRCValue(12,10,'(0):');
  SetRCValue(12,11,'(1):');
  SetRCValue(12,12,'(2):');
  SetRCValue(17,1,'BALL');
  SetRCValue(18,1,'x:');
  SetRCValue(19,1,'y:');
  SetRCValue(20,1,'z:');
  SetRCValue(17,2,'Pos:');
  SetRCValue(17,3,'[SET]');
  SetRCValue(18,3,'0.57125');
  SetRCValue(19,3,'0');
  SetRCValue(20,3,'0.1');
  SetRCValue(17,5,'MAGNET');
  SetRCValue(18,5,'Ref:');
  SetRCValue(17,7,'[ON]');
  SetRCValue(17,8,'[OFF]');
end;
