unit unit_robot;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, math, dynmatrix;

type
  TConfig = record
    l1: double;
    l2: double;
    l3: double;
    lt: double;
    H0W: TDMatrix;
    R0W: TDMatrix;
    T0W: TDMatrix;
  end;

  TJoints = record
    Pos: TDMatrix;
    Vel: TDMatrix;
    PosRef: TDMatrix;
  end;

  TTool = record
    Pos: TDMatrix;
    Rot: TDMatrix;
    PosRef: TDMatrix;
    RotRef: TDMatrix;
    PosRefFK: TDMatrix;
    RotRefFK: TDMatrix;
  end;

  TRobot = class
    public
      config: TConfig;
      JointsPrism: TJoints;
      JointsRot: TJoints;
      Tool: TTool;

      constructor Create;
      procedure FK;
      procedure FK(var JPrismMat, JRotMat: TDMatrix; var RMat, TMat: TDMatrix);
      procedure SetConfigH0W(var R, T: TDMatrix);
      procedure UpdateConfigH0W;
      procedure Stop;
    private
  end;

var
  Robot: TRobot;

function DHMat(theta, d, a, alpha: double): TDMatrix;
function HMat(R, T: TDMatrix): TDMatrix;
procedure HMat2RT(var H, R, T: TDMatrix);
procedure RT2HMat(var R, T, H: TDMatrix);
function RxMat(thetaX: double): TDMatrix;
function RyMat(thetaY: double): TDMatrix;
function RzMat(thetaZ: double): TDMatrix;

implementation

function DHMat(theta, d, a, alpha: double): TDMatrix;
begin
  Result := Meye(4);
  Result[0,0] := cos(theta); Result[0,1] := -sin(theta)*cos(alpha); Result[0,2] :=  sin(theta)*sin(alpha); Result[0,3] := a*cos(theta);
  Result[1,0] := sin(theta); Result[1,1] :=  cos(theta)*cos(alpha); Result[1,2] := -cos(theta)*sin(alpha); Result[1,3] := a*sin(theta);
  Result[2,0] :=          0; Result[2,1] :=             sin(alpha); Result[2,2] :=             cos(alpha); Result[2,3] :=            d;
end;

function HMat(R, T: TDMatrix): TDMatrix;
begin
  Result := Meye(4);
  Result[0,0] := R[0,0]; Result[0,1] := R[0,1]; Result[0,2] := R[0,2]; Result[0,3] := T[0,0];
  Result[1,0] := R[1,0]; Result[1,1] := R[1,1]; Result[1,2] := R[1,2]; Result[1,3] := T[1,0];
  Result[2,0] := R[2,0]; Result[2,1] := R[2,1]; Result[2,2] := R[2,2]; Result[2,3] := T[2,0];
end;

procedure HMat2RT(var H, R, T: TDMatrix);
begin
  // Rotation matrix
  R[0,0] := H[0,0]; R[0,1] := H[0,1]; R[0,2] := H[0,2];
  R[1,0] := H[1,0]; R[1,1] := H[1,1]; R[1,2] := H[1,2];
  R[2,0] := H[2,0]; R[2,1] := H[2,1]; R[2,2] := H[2,2];

  // Translation vector
  T[0,0] := H[0,3];
  T[1,0] := H[1,3];
  T[2,0] := H[2,3];
end;

procedure RT2HMat(var R, T, H: TDMatrix);
begin
  H[0,0] := R[0,0]; H[0,1] := R[0,1]; H[0,2] := R[0,2]; H[0,3] := T[0,0];
  H[1,0] := R[1,0]; H[1,1] := R[1,1]; H[1,2] := R[1,2]; H[1,3] := T[1,0];
  H[2,0] := R[2,0]; H[2,1] := R[2,1]; H[2,2] := R[2,2]; H[2,3] := T[2,0];
  H[3,0] :=      0; H[3,1] :=      0; H[3,2] :=      0; H[3,3] :=      1;
end;

function RxMat(thetaX: double): TDMatrix;
begin
  Result := Meye(3);
  Result[1,1] := cos(thetaX); Result[1,2] := -sin(thetaX);
  Result[2,1] := sin(thetaX); Result[2,2] :=  cos(thetaX);
end;

function RyMat(thetaY: double): TDMatrix;
begin
  Result := Meye(3);
  Result[0,0] :=  cos(thetaY); Result[0,2] := sin(thetaY);
  Result[2,0] := -sin(thetaY); Result[2,2] := cos(thetaY);
end;

function RzMat(thetaZ: double): TDMatrix;
begin
  Result := Meye(3);
  Result[0,0] := cos(thetaZ); Result[0,1] := -sin(thetaZ);
  Result[1,0] := sin(thetaZ); Result[1,1] :=  cos(thetaZ);
end;

constructor TRobot.Create;
begin
  // Initialize configuration
  config.H0W := Meye(4);
  config.R0W := Meye(3);
  config.T0W := Mzeros(3,1);

  // Initialize joints
  JointsPrism.Pos := Mzeros(1,1);
  JointsPrism.Vel := Mzeros(1,1);
  JointsPrism.PosRef := Mzeros(1,1);
  JointsRot.Pos := Mzeros(6, 1);
  JointsRot.Vel := Mzeros(6, 1);
  JointsRot.PosRef := Mzeros(6, 1);

  // Initialize tool
  Tool.Pos := Mzeros(3,1);
  Tool.Rot := Meye(3);
  Tool.PosRef := Mzeros(3,1);
  Tool.RotRef := Meye(3);
  Tool.PosRefFK := Mzeros(3,1);
  Tool.RotRefFK := Meye(3);
end;

procedure TRobot.FK;
var HTool: TDMatrix;
    H10, H21, H32, H43, H54, H65: TDMatrix;
begin
  FK(JointsPrism.Pos, JointsRot.Pos, Tool.Rot, Tool.Pos);
end;

procedure TRobot.FK(var JPrismMat, JRotMat: TDMatrix; var RMat, TMat: TDMatrix);
var HTool: TDMatrix;
    H10, H21, H32, H43, H54, H65: TDMatrix;
begin
  // DH convention
  H10 := DHMat( JRotMat[0,0]      , -config.l1 , 0         , pi/2 );
  H21 := DHMat(-JRotMat[1,0]      , 0          , config.l2 , 0    );
  H32 := DHMat(-JRotMat[2,0]+pi/2 , 0          , 0         , pi/2 );
  H43 := DHMat( JRotMat[3,0]+pi/2 , config.l3  , 0         , pi/2 );
  H54 := DHMat( JRotMat[4,0]+pi   , 0          , 0         , pi/2 );
  H65 := DHMat( JRotMat[5,0]      , config.lt  , 0         , 0    );

  // Forward Kinematics (FK): 6DoFs
  HTool := H10 * H21 * H32 * H43 * H54 * H65;

  // FK
  HMat2RT(HTool, RMat, TMat);
end;

procedure TRobot.SetConfigH0W(var R, T: TDMatrix);
begin
  config.R0W := R;
  config.T0W := T;
  UpdateConfigH0W;
end;

procedure TRobot.UpdateConfigH0W;
begin
  RT2HMat(config.R0W,config.T0W,config.H0W);
end;

procedure TRobot.Stop;
var i: Integer;
begin
  JointsPrism.PosRef[0,0] := 0;
  for i := 0 to 5 do begin
    JointsRot.PosRef[i,0] := 0;
  end;
end;

end.
