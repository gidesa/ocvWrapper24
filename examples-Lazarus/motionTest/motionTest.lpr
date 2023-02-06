program motionTest;

{$MODE Delphi}

{...$APPTYPE CONSOLE}
uses
  Interfaces,
  Forms,
  frmMotionTest in 'frmMotionTest.pas' {Form6};

{$R *.res}

begin
  Application.Initialize;
  Application.MainFormOnTaskbar := True;
  Application.CreateForm(TForm6, Form6);
  Application.Run;
end.
