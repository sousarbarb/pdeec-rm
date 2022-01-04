unit main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls,
  IniPropStorage, ComCtrls, Grids, lNetComponents, lNet, math, dynmatrix;

const
  NumJoints = 6;

type

  { TFMain }

  TFMain = class(TForm)
    BConnect: TButton;
    BDK: TButton;
    BSend: TButton;
    BIK: TButton;
    EditTheta1: TEdit;
    EditIP: TEdit;
    IniPropStorage: TIniPropStorage;
    Label1: TLabel;
    MemoMess: TMemo;
    SGPos: TStringGrid;
    StatusBar: TStatusBar;
    SGJoints: TStringGrid;
    UDP: TLUDPComponent;
    procedure BConnectClick(Sender: TObject);
    procedure BSendClick(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure UDPError(const msg: string; aSocket: TLSocket);
    procedure UDPReceive(aSocket: TLSocket);
  private
    procedure JointRefsvFromSG(var JR: TDMatrix; SG: TStringGrid);
    procedure parseMessage(var M: TDMatrix; msg: string);
    procedure SendJointRefs(var JR: TDMatrix);

  public

    JointPos, JointPosRef: TDMatrix; //array[0..NumJoints - 1] of double;

  end;

var
  FMain: TFMain;

implementation

{$R *.lfm}

{ TFMain }

procedure TFMain.BConnectClick(Sender: TObject);
begin
  UDP.Listen(9809);
end;

procedure TFMain.BSendClick(Sender: TObject);
var mess: TStringList;
    i: integer;
begin
  //JointPosRef[0, 0] := DegToRad(StrToFloat(EditTheta1.Text));

  for i := 0 to NumJoints -1 do begin
    JointPosRef[i, 0] := DegToRad(StrToFloatDef(SGJoints.Cells[2, i + 1], 0));
  end;

  mess := TStringList.create;
  try
    for i := 0 to NumJoints -1 do begin
      mess.add(format('%.4g',[JointPosRef[i, 0]]));
    end;

    UDP.SendMessage(mess.text, '127.0.0.1:9808');

  finally
    mess.free;
  end;
end;

procedure TFMain.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  SGJoints.SaveToFile('joints.txt');
end;

procedure TFMain.FormCreate(Sender: TObject);
begin
  JointPos := Mzeros(6, 1);
  JointPosRef := Mzeros(6, 1);
end;

procedure TFMain.FormShow(Sender: TObject);
begin
  if FileExists('joints.txt') then begin
    SGJoints.LoadFromFile('joints.txt');
  end;
end;

procedure TFMain.UDPError(const msg: string; aSocket: TLSocket);
begin
  StatusBar.SimpleText := msg;
end;

procedure TFMain.UDPReceive(aSocket: TLSocket);
var msg: string;
begin
  UDP.GetMessage(msg);
  //MemoMess.Text := msg;
  MemoMess.Clear;
  parseMessage(JointPos, msg);

  // control
  // ...

  // Send reference
  JointRefsvFromSG(JointPosRef, SGJoints);
  SendJointRefs(JointPosRef);
end;


procedure TFMain.parseMessage(var M: TDMatrix; msg: string);
var mess: TStringList;
    i: integer;
begin
  mess := TStringList.create;
  try
    mess.Text := msg;
    if mess.Count < NumJoints then exit;

    for i := 0 to NumJoints - 1 do begin
      M[i, 0] := StrToFloatDef(mess.Strings[i], 0);
      SGJoints.Cells[1, i + 1] := format('%.4g',[M[i, 0]]);
    end;

  finally
    mess.free;
  end;
end;


procedure TFMain.JointRefsvFromSG(var JR: TDMatrix; SG: TStringGrid);
var i: integer;
begin
  for i := 0 to NumJoints -1 do begin
    JR[i, 0] := DegToRad(StrToFloatDef(SG.Cells[2, i + 1], 0));
  end;
end;


procedure TFMain.SendJointRefs(var JR: TDMatrix);
var mess: TStringList;
    i: integer;
begin
  mess := TStringList.create;
  try
    for i := 0 to NumJoints -1 do begin
      mess.add(format('%.4g',[JR[i, 0]]));
    end;

    UDP.SendMessage(mess.text, '127.0.0.1:9808');

  finally
    mess.free;
  end;
end;


end.

