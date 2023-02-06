{ Test for Opencv  Datamatrix capture class  with ocvWrapper using webcam

  Copyright (C) 2023 Giandomenico De Sanctis gidesay@yahoo.com

  This source is free software; you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This code is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  A copy of the GNU General Public License is available on the World Wide Web
  at <http://www.gnu.org/copyleft/gpl.html>. You can also obtain it by writing
  to the Free Software Foundation, Inc., 51 Franklin Street - Fifth Floor,
  Boston, MA 02110-1335, USA.
}
unit frmDatamatrixTest;

{$MODE Delphi}

interface


uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics,
  Controls, Forms, Dialogs,
  OPENCVWrapper,
  StdCtrls;


type
  TForm6 = class(TForm)
    btnStart: TButton;
    lbl1: TLabel;
    procedure btnStartClick(Sender: TObject);
  private
    { Private declarations }
  public
    { Public declarations }
  end;

var
  Form6: TForm6;

implementation

{$R *.lfm}


const
  window_name = 'Datamatrix detection';



procedure TForm6.btnStartClick(Sender: TObject);
var
    winame: CvString_t;
    capture: PCvVideoCapture_t;
    c: Integer;
    frame: PCvMat_t;
    framegray: PCvMat_t;
    corners: PCvMat_t;
    codes: PCvvector_string;
    vecempty: PCvvector_Mat;
begin
try
  btnStart.Enabled:=False;
  winame.pstr:= PAnsiChar(AnsiString(window_name));

  capture:=pCvVideoCaptureCreate();
  frame:=pCvMatCreateEmpty();
  framegray:=pCvMatCreateEmpty();
  corners:=pCvMatCreateEmpty();
  codes:=pCvVectorstringCreate(0);
  vecempty:=pCvVectorMatCreate(0);

  pCvVideoCaptureopen2(capture, 0);
  if not(pCvVideoCaptureisOpened(capture)) then Exit;
  while True do
  begin
     if not(pCvVideoCaptureread(capture, frame)) then Break;

     pCvcvtColor(frame, framegray, COLOR_BGR2GRAY, 0);

     pCvequalizeHist(framegray, framegray);

     pCvfindDataMatrix(framegray, codes, corners, vecempty);
     pCvdrawDataMatrixCodes(frame, codes, corners);


     pCvimshow(@winame, frame );
     c:=pCvwaitKey(10);
     if Char(c) = 'c' then Break;
  end;
  pCvdestroyWindow(@winame);
  btnStart.Enabled:=True;
finally
  pCvVideoCaptureDelete(capture);
  pCvMatDelete(frame);
  pCvMatDelete(framegray);
  pCvMatDelete(corners);
  pCvVectorstringDelete(codes);
  pCvVectorMatDelete(vecempty);
end;

end;

end.
