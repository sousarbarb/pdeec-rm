unit unit_robot;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, math, dynmatrix;

type
  TJoints = record
    Pos: TDMatrix;
    Vel: TDMatrix;
    PosRef: TDMatrix;
  end;

  TRobot = class
    public
      JointsPrism: TJoints;
      JointsRot: TJoints;

      constructor Create;
      procedure Stop;
    private
  end;

var
  Robot: TRobot;

function DHMat(theta, d, a, alpha: double): TDMatrix;
function HMat(R, T: TDMatrix): TDMatrix;
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
  JointsPrism.Pos := Mzeros(1,1);
  JointsPrism.Vel := Mzeros(1,1);
  JointsPrism.PosRef := Mzeros(1,1);
  JointsRot.Pos := Mzeros(6, 1);
  JointsRot.Vel := Mzeros(6, 1);
  JointsRot.PosRef := Mzeros(6, 1);
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

