unit unit_main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Forms, Controls, Graphics, Dialogs, StdCtrls, ExtCtrls,
  ComCtrls, IniPropStorage, Grids, lNetComponents;

type

  { TFMain }

  TFMain = class(TForm)
    BtCommsConnect: TButton;
    BtIKset: TButton;
    BtConfigSet: TButton;
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
    StringGrid1: TStringGrid;
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
  private

  public

  end;

var
  FMain: TFMain;

implementation

{$R *.lfm}

end.

