unit unit_main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls, ExtCtrls,
  ComCtrls, IniPropStorage, Grids, lNetComponents, lNet, math, dynmatrix;

type

  { TFMain }

  TFMain = class(TForm)
    BtCommsConnect: TButton;
    BtIKset: TButton;
    BtConfigSet: TButton;
    CbCommsDbgClear: TButton;
    CbDebug: TCheckBox;
    CbCommsDgRx: TCheckBox;
    CbCommsDgTx: TCheckBox;
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
    GbKinematicsInverse: TGroupBox;
    GbKinematicsConfig: TGroupBox;
    GbKinematicsJoints: TGroupBox;
    GbKinematicsForward: TGroupBox;
    GbFKactual: TGroupBox;
    GbFKref: TGroupBox;
    IniPropStorage: TIniPropStorage;
    LbIKRt: TLabel;
    LbConfigR0w: TLabel;
    LbIKRtRx: TLabel;
    LbConfigR0wRx: TLabel;
    LbIKRtRy: TLabel;
    LbConfigR0wRy: TLabel;
    LbIKRtRz: TLabel;
    LbFKactXt: TLabel;
    LbConfigR0wRz: TLabel;
    LbIKXt: TLabel;
    LbFKactYt: TLabel;
    LbConfigT0wX: TLabel;
    LbConfigL1: TLabel;
    LbConfigLt: TLabel;
    LbIKYt: TLabel;
    LbFKactZt: TLabel;
    LbFKactRt: TLabel;
    LbConfigT0wY: TLabel;
    LbConfigL2: TLabel;
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
    SgFKactRt: TStringGrid;
    SgIKRt: TStringGrid;
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
    procedure CbCommsDbgClearClick(Sender: TObject);
    procedure CbCommsDgRxChange(Sender: TObject);
    procedure CbCommsDgTxChange(Sender: TObject);
    procedure CbDebugChange(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure UDPError(const msg: string; aSocket: TLSocket);
    procedure UDPReceive(aSocket: TLSocket);
  private

  public
    procedure AddMemoMessage(Memo: TMemo; msg: String);
    procedure Control;
    procedure ControlStop;
    procedure ControlManual;
    procedure ParseUDPMessage(var msg: String; var JPos, JVel: TDMatrix);
    procedure SendUDPMessage(var JRef: TDMatrix);
    procedure UpdateGUI;

  public
    UDPipS2, UDPstrS2: String;
    UDPportS2: Integer;
    UDPportLz: Integer;

  end;

const
  NUMJOINTS = 7;

var
  FMain: TFMain;

  JointPos, JointVel, JointPosRef: TDMatrix;

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
  JointPos := Mzeros(NUMJOINTS, 1);
  JointVel := Mzeros(NUMJOINTS, 1);
  JointPosRef := Mzeros(NUMJOINTS, 1);
end;

procedure TFMain.UDPError(const msg: string; aSocket: TLSocket);
begin
  StatusBar.SimpleText := msg;
end;

procedure TFMain.UDPReceive(aSocket: TLSocket);
var msg, dbgMsg: String;
    i: Integer;
begin
  // Receive message
  UDP.GetMessage(msg);

  // Parse message
  ParseUDPMessage(msg, JointPos, JointVel);

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
  // Control Mode
  if (RbModeStop.Checked) then begin
    ControlStop;
  end else if (RbModeManual.Checked) then begin
    ControlManual;
  end;

  // Send joints reference
  SendUDPMessage(JointPosRef);

  // Debug: Update GUI
  if (CbDebug.Checked) then
    UpdateGUI;
end;

procedure TFMain.ControlStop;
var i: Integer;
begin
  for i := 0 to NUMJOINTS -1 do begin
    JointPosRef[i, 0] := 0;
  end;
end;

procedure TFMain.ControlManual;
begin

end;

procedure TFMain.ParseUDPMessage(var msg: String; var JPos, JVel: TDMatrix);
var mess: TStringList;
    dbgMsg: String;
    i: Integer;
begin
  mess := TStringList.create;
  try
    mess.Text := msg;

    // Check number of messages vs expected
    //if mess.Count < NUMJOINTS then exit;

    // Current joint values
    for i := 0 to NUMJOINTS - 1 do begin
      JPos[i,0] := StrToFloatDef(mess.Strings[i], 0);
    end;

    // Current joint velocities

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

procedure TFMain.SendUDPMessage(var JRef: TDMatrix);
var mess: TStringList;
    dbgMsg: String;
    i: integer;
begin
  mess := TStringList.create;
  try
    for i := 0 to NUMJOINTS -1 do begin
      mess.add(format('%.4g',[JRef[i,0]]));
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
var i: Integer;
begin
  // Joints 
  SgKinJoints.Cells[1,1] := format('%.6g',[JointPos[0,0]]);
  SgKinJoints.Cells[2,1] := format('%.6g',[JointVel[0,0]]);
  SgKinJoints.Cells[3,1] := format('%.6g',[JointPosRef[0,0]]);
  for i := 1 to NUMJOINTS -1 do begin
    SgKinJoints.Cells[1,1+i] := format('%.6g',[RadToDeg(JointPos[i,0])]);
    SgKinJoints.Cells[2,1+i] := format('%.6g',[RadToDeg(JointVel[i,0])]);
    SgKinJoints.Cells[3,1+i] := format('%.6g',[RadToDeg(JointPosRef[i,0])]);
  end;
end;

end.

