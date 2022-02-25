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
    HW0: TDMatrix;
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
    WPos: TDMatrix;
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
      solenoid: Boolean;

      constructor Create;
      procedure FK;
      procedure FK(var JPrismMat, JRotMat: TDMatrix; var RMat, TMat: TDMatrix);
      procedure IK(elbow_up: boolean);
      function IsStopped: boolean;
      procedure SetConfigH0W(var R, T: TDMatrix);
      procedure UpdateConfigH0W;
      procedure UpdateConfigHW0;
      procedure Stop;
    private
  end;

  TPickBallSM = class
    public
      state: integer;
      pick_ball, throw_ball: boolean;

    private
      X0, Xa, Xf: TDMatrix;

    public
      constructor Create;
      procedure Update(var Rob: TRobot; var Tool, Ball: TDMatrix);
      procedure ResetState;

    private
      procedure UpdateState(var Rob: TRobot; var Tool, Ball: TDMatrix);
      procedure ExecuteState(var Rob: TRobot; var Tool, Ball: TDMatrix);
      procedure ExecuteStateInit(var Rob: TRobot; var Tool, Ball: TDMatrix);
      procedure ExecuteStateGoPickBall(var Rob: TRobot; var Tool, Ball: TDMatrix);
      procedure ExecuteStatePickBall(var Rob: TRobot; var Tool, Ball: TDMatrix);
      procedure ExecuteStateSetThrowDir(var Rob: TRobot; var Tool, Ball: TDMatrix);
      procedure ExecuteStateThrowBall(var Rob: TRobot; var Tool, Ball: TDMatrix);
      procedure ExecuteStateThrowBallDelay(var Rob: TRobot; var Tool, Ball: TDMatrix);
      procedure ExecuteStateGoBack(var Rob: TRobot; var Tool, Ball: TDMatrix);
  end;

const
  STATE_INIT             = 0;
  STATE_GO_PICK_BALL     = 1;
  STATE_PICK_BALL        = 2;
  STATE_SET_THROW_DIR    = 3;
  STATE_THROW_BALL       = 4;
  STATE_THROW_BALL_DELAY = 5;
  STATE_GO_BACK          = 6;

var
  Robot: TRobot;

function DHMat(theta, d, a, alpha: double): TDMatrix;
function HMat(R, T: TDMatrix): TDMatrix;
procedure HMat2RT(var H, R, T: TDMatrix);
procedure RT2HMat(var R, T, H: TDMatrix);
function RxMat(thetaX: double): TDMatrix;
function RyMat(thetaY: double): TDMatrix;
function RzMat(thetaZ: double): TDMatrix;

procedure InterpolatePosition(var X0: TDMatrix; vnom, dt: double; var Xf, Xa: TDMatrix);
procedure InterpolatePosition(var X0: TDMatrix; vnom, dt: double; var Xf: TDMatrix);

function VNorm2(V: TDMatrix): double;

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

procedure InterpolatePosition(var X0: TDMatrix; vnom, dt: double; var Xf, Xa: TDMatrix);
var DX: TDMatrix;
    DXnorm: double;
begin
  DXnorm := vnom * dt / VNorm2(Xf - X0);
  DX := (Xf - X0) * DXnorm;

  if (VNorm2(Xf - Xa) <= vnom * dt) then begin
    Xa := Xf;
  end else begin
    Xa := Xa + DX;
  end;
end;

procedure InterpolatePosition(var X0: TDMatrix; vnom, dt: double; var Xf: TDMatrix);
begin
  InterpolatePosition(X0, vnom, dt, Xf, X0);
end;

function VNorm2(V: TDMatrix): double;
var i: Integer;
begin
  Result := 0;
  for i:=0 to V.NumRows do begin
    Result := Result + Sqr(V[i,0]);
  end;
  Result := Sqrt(Result);
end;

constructor TRobot.Create;
begin
  // Initialize configuration
  config.H0W := Meye(4);
  config.HW0 := Meye(4);
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
  Tool.WPos := Mzeros(3,1);
  Tool.Rot := Meye(3);
  Tool.PosRef := Mzeros(3,1);
  Tool.RotRef := Meye(3);
  Tool.PosRefFK := Mzeros(3,1);
  Tool.RotRefFK := Meye(3);

  // Initialize solenoid
  solenoid := false;
end;

procedure TRobot.FK;
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

procedure TRobot.IK(elbow_up: boolean);
var ToolLength, WristPosRef, WristRotRef: TDMatrix;
    H10, H21, H32, H30, R30, T30: TDMatrix;
    s, r, cth3: double;
begin
  // Initialization
  ToolLength := Mzeros(3,1);
  ToolLength[2,0] := config.lt;
  R30 := Meye(3);
  T30 := Mzeros(3,1);

  // Inverse Kinematics: Central Point of the Wrist
  // - central point
  WristPosRef := Tool.PosRef - Tool.RotRef * ToolLength;

  // - joints 1-3 (reference)
  // (J1)
  if ((WristPosRef[0,0] <> 0) AND (WristPosRef[1,0] <> 0)) then
    JointsRot.PosRef[0,0] := ArcTan2(WristPosRef[1,0],WristPosRef[0,0]);
    // or JointsRot.PosRef[0,0] := pi + ArcTan2(WristPosRef[1,0],WristPosRef[0,0]);
  // (J3)
  s := WristPosRef[2,0] + config.l1;
  r := Sqrt(Sqr(WristPosRef[0,0]) + Sqr(WristPosRef[1,0]));
  cth3 := (Sqr(s) + Sqr(r) - Sqr(config.l2) - Sqr(config.l3)) / (2*config.l2*config.l3);
  if (1-Sqr(cth3) < 0) then
    raise Exception.Create(Format('Pose not possible due to 1 - sqrt(cth3) = %.6g < 0 (outside of the robot work volume)',
                                  [1-Sqr(cth3)]));
  if (NOT elbow_up) then begin
    JointsRot.PosRef[2,0] := ArcTan2(Sqrt(1-Sqr(cth3)),cth3);
  end else begin
    JointsRot.PosRef[2,0] := ArcTan2(-Sqrt(1-Sqr(cth3)),cth3);
  end;
  // (J2)
  JointsRot.PosRef[1,0] :=
    ArcTan2(s,r) -
    ArcTan2(config.l3*sin(JointsRot.PosRef[2,0]), config.l2 + config.l3*cos(JointsRot.PosRef[2,0]));
  // (-J2,-J3) due to SimTwo
  JointsRot.PosRef[1,0] := -JointsRot.PosRef[1,0];
  JointsRot.PosRef[2,0] := -JointsRot.PosRef[2,0];

  // - wrist orientation
  // DH convention
  H10 := DHMat( JointsRot.PosRef[0,0]      , -config.l1 , 0         , pi/2 );
  H21 := DHMat(-JointsRot.PosRef[1,0]      , 0          , config.l2 , 0    );
  H32 := DHMat(-JointsRot.PosRef[2,0]+pi/2 , 0          , 0         , pi/2 );
  H30 := H10 * H21 * H32;
  HMat2RT(H30,R30,T30);
  WristRotRef := Mtran(R30) * Tool.RotRef;

  // - joints 4-6 (reference)
  // (J5)
  JointsRot.PosRef[4,0] := ArcTan2(Sqrt(1-WristRotRef[2,2]),WristRotRef[2,2]);
  // or JointsRot.PosRef[4,0] := ArcTan2(-Sqrt(1-WristRotRef[2,2]),WristRotRef[2,2]);
  // (J4,J6)
  if (WristRotRef[2,2] <> 0) then begin
    JointsRot.PosRef[3,0] := ArcTan2(WristRotRef[0,2],-WristRotRef[1,2]);
    JointsRot.PosRef[5,0] := ArcTan2(WristRotRef[2,1],-WristRotRef[2,0]);
  end else begin
    JointsRot.PosRef[3,0] := 0;
    if (WristRotRef[2,2] = 1) then begin
      JointsRot.PosRef[5,0] :=  ArcTan2( WristRotRef[0,0],WristRotRef[0,1]);
    end else begin
      JointsRot.PosRef[5,0] := -ArcTan2(-WristRotRef[0,0],WristRotRef[0,1]);
    end;
    // or infinite solutions for th4 + th6 or th4 - th6
  end;
end;

function TRobot.IsStopped: boolean;
const STOP_VEL_LIN = 0.01;
      STOP_VEL_ANG = 1 * Pi / 180;
var i: integer;
begin
  Result := true;

  // Check velocity of the joints
  if (Abs(JointsPrism.Vel[0,0]) > STOP_VEL_LIN) then
    Result := false;
  for i := 0 to 5 do begin
    if (Abs(JointsRot.Vel[0,0]) > STOP_VEL_ANG) then
      Result := false;
  end;
end;

procedure TRobot.SetConfigH0W(var R, T: TDMatrix);
begin
  config.R0W := R;
  config.T0W := T;
end;

procedure TRobot.UpdateConfigH0W;
begin
  RT2HMat(config.R0W,config.T0W,config.H0W);
end;

procedure TRobot.UpdateConfigHW0;
begin

  config.HW0 := config.H0W;
  config.HW0[0,3] := -config.HW0[0,3];
  config.HW0[1,3] := -config.HW0[1,3];
  config.HW0[2,3] := -config.HW0[2,3];

end;

procedure TRobot.Stop;
var i: Integer;
begin
  JointsPrism.PosRef[0,0] := 0;
  for i := 0 to 5 do begin
    JointsRot.PosRef[i,0] := 0;
  end;
end;

constructor TPickBallSM.Create;
begin
  // Reset state machine
  ResetState;

  // Initialize matrices
  X0 := Mzeros(3,1);
  Xa := Mzeros(3,1);
  Xf := Mzeros(3,1);
end;

procedure TPickBallSM.Update(var Rob: TRobot; var Tool, Ball: TDMatrix);
begin
  // Change state
  UpdateState(Rob, Tool, Ball);

  // Execute actions given a certain state
  ExecuteState(Rob, Tool, Ball);
end;

procedure TPickBallSM.ResetState;
begin
  // Reset state
  state := STATE_INIT;

  // Reset other variable
  pick_ball  := false;
  throw_ball := false;
end;

procedure TPickBallSM.UpdateState(var Rob: TRobot; var Tool, Ball: TDMatrix);
const DIST_APPROACH_BALL = 0.02;
      MAX_LIM_Q0 = 0.57125;
      THROW_DELAY_RATIO = 0.20;
      THROW_LIMIT = 0.05;
begin
  case (state) of

    STATE_INIT: begin
      if (pick_ball) then begin
        pick_ball := false;
        X0 := Tool;
        Xa := X0;
        Xf := Ball;
        Xf[2,0] := Xf[2,0] + DIST_APPROACH_BALL;

        state := STATE_GO_PICK_BALL;
      end;
    end;

    STATE_GO_PICK_BALL: begin
      if (VNorm2(Tool - Xf) < DIST_APPROACH_BALL*2) then begin
        state := STATE_PICK_BALL;
      end;
    end;

    STATE_PICK_BALL: begin
      if (Rob.solenoid) then begin  // TODO: maybe distance ball - tool < DIST_APPROACH_BALL / 2?
        state := STATE_SET_THROW_DIR;
      end;
    end;

    STATE_SET_THROW_DIR: begin
      if (throw_ball) then begin
        throw_ball := false;

        state := STATE_THROW_BALL;
      end;
    end;

    STATE_THROW_BALL: begin
      if (Rob.JointsPrism.Pos[0,0] > (1-THROW_DELAY_RATIO)*MAX_LIM_Q0) then begin
        state := STATE_THROW_BALL_DELAY;
      end;
    end;

    STATE_THROW_BALL_DELAY: begin
      if (Rob.JointsPrism.Pos[0,0] > (1-THROW_LIMIT)*MAX_LIM_Q0) then begin
        {X0 := Rob.Tool.Pos; // TODO: now relative to local frame
        Xa := X0;
        Xf := Mzeros(3,1);
        Xf[0,0] := 0.805;
        Xf[2,0] := -0.30;}

        state := STATE_GO_BACK;
      end;
    end;

    STATE_GO_BACK: begin
      if (Rob.IsStopped) then begin  // TODO: maybe isstopped + desired posed reached?
        state := STATE_INIT;
      end;
    end;

    else begin
      ResetState;
    end;

  end;
end;

procedure TPickBallSM.ExecuteState(var Rob: TRobot; var Tool, Ball: TDMatrix);
begin
  case (state) of

    STATE_INIT: begin
      ExecuteStateInit(Rob, Tool, Ball);
    end;

    STATE_GO_PICK_BALL: begin
      ExecuteStateGoPickBall(Rob, Tool, Ball);
    end;

    STATE_PICK_BALL: begin
      ExecuteStatePickBall(Rob, Tool, Ball);
    end;

    STATE_SET_THROW_DIR: begin
      ExecuteStateSetThrowDir(Rob, Tool, Ball);
    end;

    STATE_THROW_BALL: begin
      ExecuteStateThrowBall(Rob, Tool, Ball);
    end;

    STATE_THROW_BALL_DELAY: begin
      ExecuteStateThrowBallDelay(Rob, Tool, Ball);
    end;

    STATE_GO_BACK: begin
      ExecuteStateGoBack(Rob, Tool, Ball);
    end;

    else begin
      ResetState;
    end;

  end;
end;

procedure TPickBallSM.ExecuteStateInit(var Rob: TRobot; var Tool, Ball: TDMatrix);
begin
  // Stop Robot
  Rob.Stop;

  // Solenoid
  Rob.solenoid := false;

  // q0: prismatic joint
  Rob.JointsPrism.PosRef[0,0] := 0;
end;

procedure TPickBallSM.ExecuteStateGoPickBall(var Rob: TRobot; var Tool, Ball: TDMatrix);
begin
  // Solenoid
  Rob.solenoid := false;

  // TODO: Position
  // put here linear interpolation for the robot's tool reference
  // InterpolatePosition(X0, 0.05, 0.40, Xf, Xa);
  // ToolRefW := Xa;

  // Orientation
  Rob.Tool.RotRef := RzMat(DegToRad(90.0)) * RyMat(DegToRad(0.0)) * RxMat(DegToRad(180.0));

  // Inverse kinematics (probably)

  // q0: prismatic joint
  Rob.JointsPrism.PosRef[0,0] := 0;
end;

procedure TPickBallSM.ExecuteStatePickBall(var Rob: TRobot; var Tool, Ball: TDMatrix);
begin
  // Solenoid
  Rob.solenoid := true;

  // TODO: Position
  // ToolRefW := Xf;

  // Orientation
  Rob.Tool.RotRef := RzMat(DegToRad(90.0)) * RyMat(DegToRad(0.0)) * RxMat(DegToRad(180.0));

  // Inverse kinematics (probably)

  // q0: prismatic joint
  Rob.JointsPrism.PosRef[0,0] := 0;
end;

procedure TPickBallSM.ExecuteStateSetThrowDir(var Rob: TRobot; var Tool, Ball: TDMatrix);
begin
  // Solenoid
  Rob.solenoid := true;

  // Orientation
  Rob.Tool.RotRef := RzMat(DegToRad(90.0)) * RyMat(DegToRad(0.0)) * RxMat(DegToRad(180.0));

  // DO NOT CHANGE REFERENCE OF THE TOOL POSITION (manually changed)
end;

procedure TPickBallSM.ExecuteStateThrowBall(var Rob: TRobot; var Tool, Ball: TDMatrix);
begin
  // Solenoid
  Rob.solenoid := true;

  // Orientation
  Rob.Tool.RotRef := RzMat(DegToRad(90.0)) * RyMat(DegToRad(0.0)) * RxMat(DegToRad(180.0));

  // TODO: change directly q0 (incrementing e.g.)
  // Rob.JointsPrism.PosRef[0,0] := Rob.JointsPrism.PosRef[0,0] + VNOM * DT;
end;

procedure TPickBallSM.ExecuteStateThrowBallDelay(var Rob: TRobot; var Tool, Ball: TDMatrix);
begin
  // Solenoid
  Rob.solenoid := false;

  // Orientation
  Rob.Tool.RotRef := RzMat(DegToRad(90.0)) * RyMat(DegToRad(0.0)) * RxMat(DegToRad(180.0));

  // TODO: change directly q0 (keep incrementing e.g.)
  // Rob.JointsPrism.PosRef[0,0] := Rob.JointsPrism.PosRef[0,0] + VNOM * DT;
end;

procedure TPickBallSM.ExecuteStateGoBack(var Rob: TRobot; var Tool, Ball: TDMatrix);
begin
  // Solenoid
  Rob.solenoid := false;

  // TODO: Position
  // put here linear interpolation for the robot's tool reference
  // InterpolatePosition(X0, 0.05, 0.40, Xf, Xa);
  // ToolRefW := Xa;

  // Orientation
  Rob.Tool.RotRef := RzMat(DegToRad(90.0)) * RyMat(DegToRad(0.0)) * RxMat(DegToRad(90.0));

  // Inverse kinematics (probably)

  // q0: prismatic joint
  Rob.JointsPrism.PosRef[0,0] := 0
end;

end.

