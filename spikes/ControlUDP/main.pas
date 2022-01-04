unit main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls,
  IniPropStorage, ComCtrls, lNetComponents, lNet, math;

const
  NumJoints = 6;

type

  { TFMain }

  TFMain = class(TForm)
    BConnect: TButton;
    BSend: TButton;
    EditTheta1: TEdit;
    EditIP: TEdit;
    IniPropStorage: TIniPropStorage;
    Label1: TLabel;
    MemoMess: TMemo;
    StatusBar: TStatusBar;
    UDP: TLUDPComponent;
    procedure BConnectClick(Sender: TObject);
    procedure BSendClick(Sender: TObject);
    procedure UDPError(const msg: string; aSocket: TLSocket);
    procedure UDPReceive(aSocket: TLSocket);
  private

  public

    JointPos: array[0..NumJoints - 1] of double;

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
  JointPos[0] := DegToRad(StrToFloat(EditTheta1.Text));

  mess := TStringList.create;
  try
    for i := 0 to NumJoints -1 do begin
      mess.add(format('%.4g',[JointPos[i]]));
    end;

    UDP.SendMessage(mess.text, '127.0.0.1:9808');

  finally
    mess.free;
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
  MemoMess.Text := msg;
end;

end.

