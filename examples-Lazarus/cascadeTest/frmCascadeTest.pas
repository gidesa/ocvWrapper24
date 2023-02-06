{ Test for Opencv face detector class with ocvWrapper, using webcam

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
unit frmCascadeTest;

{$MODE Delphi}

interface

uses StdCtrls,
  Windows, Messages, SysUtils, Variants, Classes, Graphics,
  Controls, Forms, Dialogs,
  OPENCVWrapper;


type
  TForm6 = class(TForm)
    btnStart: TButton;
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
  face_cascade_name = '..\\..\\haarcascade_frontalface_alt2.xml';
  window_name = 'Capture - Face detection';



procedure TForm6.btnStartClick(Sender: TObject);
var
    i: Integer;
    face_cascade: PCvCascadeClassifier_t;
    filename, winame: CvString_t;
    capture: PCvVideoCapture_t;
    c: Integer;
    frame: PCvMat_t;
    faces: PCvvector_Rect;
    nrfaces: Integer;
    framegray: PCvMat_t;
    pminsize: PCvSize_t;
    paxes: PCvSize_t;
    psizeempty: PCvSize_t;
    ppoint: PCvPoint_t;
    pcolor: PCvScalar_t;

    prect: PCvRect_t;
    crect: CvRectS;
begin
try
  btnStart.Enabled:=False;
  face_cascade:=pCvCascadeClassifierCreate();
  filename.pstr:=PAnsiChar(AnsiString(face_cascade_name));
  winame.pstr:= PAnsiChar(AnsiString(window_name));

  capture:=pCvVideoCaptureCreate();
  frame:=pCvMatCreateEmpty();
  framegray:=pCvMatCreateEmpty();
  faces:=pCvVectorRectCreate(0);
  pminsize:=CvSize_(30, 30);
  ppoint:=pCvPointCreate();
  pcolor:=CvScalar_(255,0,255,0);
  psizeempty:=pCvSizeCreate();
  paxes:=pCvSizeCreate();
  if not(pCvCascadeClassifierload(face_cascade, @filename)) then
  begin
      ShowMessage('Error loading');
      Exit;
  end;
  pCvVideoCaptureopen2(capture, 0);
  if not(pCvVideoCaptureisOpened(capture)) then Exit;
  while True do
  begin
     if not(pCvVideoCaptureread(capture, frame)) then Break;

     pCvcvtColor(frame, framegray, COLOR_BGR2GRAY, 0);
     pCvequalizeHist(framegray, framegray);
     pCvCascadeClassifierdetectMultiScale(face_cascade, framegray, faces, 1.1, 2, (0 or CV_HAAR_SCALE_IMAGE),
                    pminsize, psizeempty);
     nrfaces:=pCvVectorRectLength(faces);
     for  i:=0 to nrfaces-1 do
     begin
      prect:=pCvVectorRectGet(faces, i);
      pCvRectToStruct(prect, @crect);
      CvPoint_(crect.x + crect.width shr 1, crect.y + crect.height shr 1, ppoint);
      CvSize_(crect.width shr 1, crect.height shr 1 ,  paxes);

      pCvellipse(frame, ppoint, paxes, 0, 0, 360, pcolor, 2, 8, 0);
      pCvRectDelete(prect);
     end;


     pCvimshow(@winame, frame );
     c:=pCvwaitKey(10);
     if Char(c) = 'c' then Break;
  end;
  pCvdestroyWindow(@winame);
  btnStart.Enabled:=True;
finally
  pCvVideoCaptureDelete(capture);
  pCvCascadeClassifierDelete(face_cascade);
  pCvMatDelete(frame);
  pCvVectorRectDelete(faces);
  pCvMatDelete(framegray);
  pCvSizeDelete(pminsize);
  pCvSizeDelete(paxes);
  pCvPointDelete(ppoint);
  pCvScalarDelete(pcolor);
  pCvSizeDelete(psizeempty);
end;

end;

end.
