unit unit_main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls, ExtCtrls,
  ComCtrls, IniPropStorage, Grids, lNetComponents, lNet, math, dynmatrix,
  unit_robot;

type

  { TFMain }

  TFMain = class(TForm)
    BtCommsConnect: TButton;
    BtIKset: TButton;
    BtConfigSet: TButton;
    BtIKreset: TButton;
    BtJointsRefSet: TButton;
    BtJointsRefReset: TButton;
    CbCommsDbgClear: TButton;
    CbDebug: TCheckBox;
    CbCommsDgRx: TCheckBox;
    CbCommsDgTx: TCheckBox;
    CbIKelbowUp: TCheckBox;
    EdCommsIPS2: TEdit;
    EdCommsPortS2: TEdit;
    EdCommsPortLz: TEdit;
    EdFKactXt: TEdit;
    EdConfigR0wRx: TEdit;
    EdConfigR0wRy: TEdit;
    EdConfigR0wRz: TEdit;
    EdIKXt: TEdit;
    EdIKRtRx: TEdit;
    EdFKactYt: TEdit;
    EdConfigT0wX: TEdit;
    EdConfigL1: TEdit;
    EdConfigLt: TEdit;
    EdJointsRefQ0: TEdit;
    EdIKYt: TEdit;
    EdIKRtRy: TEdit;
    EdFKactZt: TEdit;
    EdConfigT0wY: TEdit;
    EdConfigL2: TEdit;
    EdIKZt: TEdit;
    EdIKRtRz: TEdit;
    EdFKrefXt: TEdit;
    EdFKrefYt: TEdit;
    EdFKrefZt: TEdit;
    EdConfigT0wZ: TEdit;
    EdConfigL3: TEdit;
    EdJointsRefQ1: TEdit;
    EdJointsRefQ2: TEdit;
    EdJointsRefQ3: TEdit;
    EdJointsRefQ4: TEdit;
    EdJointsRefQ5: TEdit;
    EdJointsRefQ6: TEdit;
    GbKinematicsInverse: TGroupBox;
    GbKinematicsConfig: TGroupBox;
    GbKinematicsJointsRef: TGroupBox;
    GbKinematicsJoints: TGroupBox;
    GbKinematicsForward: TGroupBox;
    GbFKactual: TGroupBox;
    GbFKref: TGroupBox;
    IniPropStorage: TIniPropStorage;
    LbConfigR0w: TLabel;
    LbIKRtRx: TLabel;
    LbConfigR0wRx: TLabel;
    LbJointsRefQ3: TLabel;
    LbIKRtRy: TLabel;
    LbConfigR0wRy: TLabel;
    LbJointsRefQ4: TLabel;
    LbIKRtRz: TLabel;
    LbFKactXt: TLabel;
    LbConfigR0wRz: TLabel;
    LbJointsRefQ5: TLabel;
    LbIKXt: TLabel;
    LbFKactYt: TLabel;
    LbConfigT0wX: TLabel;
    LbConfigL1: TLabel;
    LbConfigLt: TLabel;
    LbJointsRefQ0: TLabel;
    LbIKYt: TLabel;
    LbFKactZt: TLabel;
    LbFKactRt: TLabel;
    LbConfigT0wY: TLabel;
    LbConfigL2: TLabel;
    LbJointsRefQ1: TLabel;
    LbIKZt: TLabel;
    LbFKrefXt: TLabel;
    LbFKrefYt: TLabel;
    LbFKrefZt: TLabel;
    LbFKrefRt: TLabel;
    LbCommsIPS2: TLabel;
    LbCommsPortS2: TLabel;
    LbCommsPortLz: TLabel;
    LbCommsDebug: TLabel;
    LbConfigT0wZ: TLabel;
    LbConfigL3: TLabel;
    LbJointsRefQ2: TLabel;
    LbJointsRefQ6: TLabel;
    SgFKactRt: TStringGrid;
    SgConfigR0w: TStringGrid;
    SgKinJoints: TStringGrid;
    SgFKrefRt: TStringGrid;
    TsKinematics: TTabSheet;
    UDP: TLUDPComponent;
    MmCommsDebug: TMemo;
    PcMain: TPageControl;
    RbModeStop: TRadioButton;
    RbModeManual: TRadioButton;
    RbModeBall: TRadioButton;
    RbModeFull: TRadioButton;
    RgModeCtrl: TRadioGroup;
    StatusBar: TStatusBar;
    TsComms: TTabSheet;
    procedure BtCommsConnectClick(Sender: TObject);
    procedure BtConfigSetClick(Sender: TObject);
    procedure BtIKsetClick(Sender: TObject);
    procedure BtJointsRefResetClick(Sender: TObject);
    procedure BtJointsRefSetClick(Sender: TObject);
    procedure CbCommsDbgClearClick(Sender: TObject);
    procedure CbCommsDgRxChange(Sender: TObject);
    procedure CbCommsDgTxChange(Sender: TObject);
    procedure CbDebugChange(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure RbModeManualChange(Sender: TObject);
    procedure UDPError(const msg: string; aSocket: TLSocket);
    procedure UDPReceive(aSocket: TLSocket);
  private

  public
    procedure AddMemoMessage(Memo: TMemo; msg: String);
    procedure Control;
    procedure ControlStop;
    procedure ControlManual;
    procedure ControlManualConfig(enable: boolean);
    procedure ParseUDPMessage(var msg: String);
    procedure SendUDPMessage;
    procedure UpdateGUI;

  public
    UDPipS2, UDPstrS2: String;
    UDPportS2: Integer;
    UDPportLz: Integer;

  end;

var
  FMain: TFMain;

implementation

{$R *.lfm}

{ TFMain }

procedure TFMain.BtCommsConnectClick(Sender: TObject);
begin
  UDPipS2 := EdCommsIPS2.Text;
  UDPportS2 := StrToIntDef(EdCommsPortS2.Text, 9808);
  UDPportLz := StrToIntDef(EdCommsPortLz.Text, 9809);

  UDP.Listen(UDPportLz);
  UDPstrS2 := Format('%s:%d', [UDPipS2, UDPportS2]);
end;

procedure TFMain.BtConfigSetClick(Sender: TObject);
var thx, thy, thz: double;
    i, j: Integer;
begin
  // Configuration Robot: T 0 >>> World
  Robot.config.T0W[0,0] := StrToFloatDef(EdConfigT0wX.Text,0.57125);
  Robot.config.T0W[1,0] := StrToFloatDef(EdConfigT0wY.Text,0);
  Robot.config.T0W[2,0] := StrToFloatDef(EdConfigT0wZ.Text,1.15);

  // Configuration Robot: R 0 >>> World
  thx := DegToRad(StrToFloatDef(EdConfigR0wRx.Text,0));
  thy := DegToRad(StrToFloatDef(EdConfigR0wRy.Text,0));
  thz := DegToRad(StrToFloatDef(EdConfigR0wRz.Text,0));
  Robot.config.R0W := RzMat(thz) * RyMat(thy) * RxMat(thx);
  for i := 0 to 2 do begin
    for j := 0 to 2 do begin
      SgConfigR0w.Cells[j,i] := format('%.6g',[Robot.config.R0W[i,j]]);
    end;
  end;

  // Configuration Robot: H 0 >>> World
  Robot.UpdateConfigH0W;

  // Configuration Robot: links + tools lengths
  Robot.config.l1 := StrToFloatDef(EdConfigL1.Text,0.3);
  Robot.config.l2 := StrToFloatDef(EdConfigL2.Text,0.4);
  Robot.config.l3 := StrToFloatDef(EdConfigL3.Text,0.37);
  Robot.config.lt := StrToFloatDef(EdConfigLt.Text,0.04/2 + 0.03/2);
end;

procedure TFMain.BtIKsetClick(Sender: TObject);
var thx, thy, thz: double;
begin
  // Tool Reference: Position
  Robot.Tool.PosRef[0,0] := StrToFloatDef(EdIKXt.Text,Robot.config.l2+Robot.config.l3+Robot.config.lt);
  Robot.Tool.PosRef[1,0] := StrToFloatDef(EdIKYt.Text,0);
  Robot.Tool.PosRef[2,0] := StrToFloatDef(EdIKZt.Text,-Robot.config.l1);

  // Tool Reference: Rotation
  thx := DegToRad(StrToFloatDef(EdIKRtRx.Text,90));
  thy := DegToRad(StrToFloatDef(EdIKRtRy.Text,0));
  thz := DegToRad(StrToFloatDef(EdIKRtRz.Text,90));
  Robot.Tool.RotRef := RzMat(thz) * RyMat(thy) * RxMat(thx);

  // Inverse Kinematics
  Robot.IK(CbIKelbowUp.Checked);
end;

procedure TFMain.BtJointsRefResetClick(Sender: TObject);
begin
  Robot.Stop;
end;

procedure TFMain.BtJointsRefSetClick(Sender: TObject);
begin
  Robot.JointsPrism.PosRef[0,0] := StrToFloatDef(EdJointsRefQ0.Text,0);
  Robot.JointsRot.PosRef[0,0] := DegToRad(StrToFloatDef(EdJointsRefQ1.Text,0));
  Robot.JointsRot.PosRef[1,0] := DegToRad(StrToFloatDef(EdJointsRefQ2.Text,0));
  Robot.JointsRot.PosRef[2,0] := DegToRad(StrToFloatDef(EdJointsRefQ3.Text,0));
  Robot.JointsRot.PosRef[3,0] := DegToRad(StrToFloatDef(EdJointsRefQ4.Text,0));
  Robot.JointsRot.PosRef[4,0] := DegToRad(StrToFloatDef(EdJointsRefQ5.Text,0));
  Robot.JointsRot.PosRef[5,0] := DegToRad(StrToFloatDef(EdJointsRefQ6.Text,0));
end;

procedure TFMain.CbCommsDbgClearClick(Sender: TObject);
begin
  MmCommsDebug.Clear;
end;

procedure TFMain.CbCommsDgRxChange(Sender: TObject);
begin
  if (CbCommsDgRx.Checked) then
    CbDebug.Checked := true;
end;

procedure TFMain.CbCommsDgTxChange(Sender: TObject);
begin
  if (CbCommsDgTx.Checked) then
    CbDebug.Checked := true;
end;

procedure TFMain.CbDebugChange(Sender: TObject);
begin
  if (NOT CbDebug.Checked) then begin
    CbCommsDgRx.Checked := false;
    CbCommsDgTx.Checked := false;
  end;
end;

procedure TFMain.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  UDP.Disconnect(true);
end;

procedure TFMain.FormCreate(Sender: TObject);
begin
  Robot := TRobot.Create;
end;

procedure TFMain.FormShow(Sender: TObject);
begin
  BtCommsConnect.Click;
  BtConfigSet.Click;
  RbModeManualChange(RbModeManual);
end;

procedure TFMain.RbModeManualChange(Sender: TObject);
begin
  ControlManualConfig(RbModeManual.Checked);
end;

procedure TFMain.UDPError(const msg: string; aSocket: TLSocket);
begin
  StatusBar.SimpleText := msg;
end;

procedure TFMain.UDPReceive(aSocket: TLSocket);
var msg: String;
begin
  // Receive message
  UDP.GetMessage(msg);

  // Parse message
  ParseUDPMessage(msg);

  // Control
  Control;
end;



{ CONTROL }

procedure TFMain.AddMemoMessage(Memo: TMemo; msg: String);
begin
  Memo.Lines.Add(msg);
  while Memo.Lines.Count > Memo.Tag do begin
    Memo.Lines.Delete(0);
  end;
end;

procedure TFMain.Control;
begin
  // Input data processment
  Robot.FK;

  // Control
  if (RbModeStop.Checked) then begin
    ControlStop;
  end else if (RbModeManual.Checked) then begin
    ControlManual;
  end;

  // Output joints reference
  SendUDPMessage;

  // Debug: Update GUI
  if (CbDebug.Checked) then
    UpdateGUI;
end;

procedure TFMain.ControlStop;
begin
  Robot.Stop;
end;

procedure TFMain.ControlManual;
begin

end;

procedure TFMain.ControlManualConfig(enable: boolean);
begin
  GbKinematicsJointsRef.Enabled := enable;
  GbKinematicsInverse.Enabled   := enable;
end;

procedure TFMain.ParseUDPMessage(var msg: String);
var mess: TStringList;
    dbgMsg: String;
    i: Integer;
begin
  mess := TStringList.create;
  try
    mess.Text := msg;

    // Check number of messages vs expected
    if mess.Count < 14 then exit;

    // Current joint values
    Robot.JointsPrism.Pos[0,0] := StrToFloatDef(mess.Strings[0], 0);
    for i := 0 to 5 do begin
      Robot.JointsRot.Pos[i,0] := StrToFloatDef(mess.Strings[i+1], 0);
    end;

    // Current joint velocities
    Robot.JointsPrism.Vel[0,0] := StrToFloatDef(mess.Strings[7], 0);
    for i := 0 to 5 do begin
      Robot.JointsRot.Vel[i,0] := StrToFloatDef(mess.Strings[i+8], 0);
    end;

    // Debug
    if (CbCommsDgRx.Checked) then begin
      dbgMsg := '[S2 > Lz] ';

      for i := 0 to mess.Count-1 do begin
        dbgMsg := dbgMsg + mess.Strings[i] + ' ';
      end;

      AddMemoMessage(MmCommsDebug, dbgMsg);
    end;

  finally
    mess.free;
  end;
end;

procedure TFMain.SendUDPMessage;
var mess: TStringList;
    dbgMsg: String;
    i: integer;
begin
  mess := TStringList.create;
  try
    // Reference joints value
    mess.add(format('%.4g',[Robot.JointsPrism.PosRef[0,0]]));
    for i := 0 to 5 do begin
      mess.add(format('%.4g',[Robot.JointsRot.PosRef[i,0]]));
    end;

    UDP.SendMessage(mess.text, UDPstrS2);
    if (CbCommsDgTx.Checked) then begin
      dbgMsg := '[Lz > S2] ';

      for i := 0 to mess.Count-1 do begin
        dbgMsg := dbgMsg + mess.Strings[i] + ' ';
      end;

      AddMemoMessage(MmCommsDebug, dbgMsg);
    end;

  finally
    mess.free;
  end;
end;

procedure TFMain.UpdateGUI;
var i, j: Integer;
begin
  // Joints 
  SgKinJoints.Cells[1,1] := format('%.6g',[Robot.JointsPrism.Pos[0,0]]);
  SgKinJoints.Cells[2,1] := format('%.6g',[Robot.JointsPrism.Vel[0,0]]);
  SgKinJoints.Cells[3,1] := format('%.6g',[Robot.JointsPrism.PosRef[0,0]]);
  for i := 0 to 5 do begin
    SgKinJoints.Cells[1,2+i] := format('%.6g',[RadToDeg(Robot.JointsRot.Pos[i,0])]);
    SgKinJoints.Cells[2,2+i] := format('%.6g',[Robot.JointsRot.Vel[i,0]]);
    SgKinJoints.Cells[3,2+i] := format('%.6g',[RadToDeg(Robot.JointsRot.PosRef[i,0])]);
  end;

  // Forward Kinematics
  EdFKactXt.Text := format('%.6g',[Robot.Tool.Pos[0,0]]);
  EdFKactYt.Text := format('%.6g',[Robot.Tool.Pos[1,0]]);
  EdFKactZt.Text := format('%.6g',[Robot.Tool.Pos[2,0]]);
  for i := 0 to 2 do begin
    for j := 0 to 2 do begin
      SgFKactRt.Cells[j,i] := format('%.6g',[Robot.Tool.Rot[i,j]]);
    end;
  end;

  // Inverse Kinematics
  Robot.FK(Robot.JointsPrism.PosRef, Robot.JointsRot.PosRef, Robot.Tool.RotRefFK, Robot.Tool.PosRefFK);
  EdFKrefXt.Text := format('%.6g',[Robot.Tool.PosRefFK[0,0]]);
  EdFKrefYt.Text := format('%.6g',[Robot.Tool.PosRefFK[1,0]]);
  EdFKrefZt.Text := format('%.6g',[Robot.Tool.PosRefFK[2,0]]);
  for i := 0 to 2 do begin
    for j := 0 to 2 do begin
      SgFKrefRt.Cells[j,i] := format('%.6g',[Robot.Tool.RotRefFK[i,j]]);
    end;
  end;
end;

end.

