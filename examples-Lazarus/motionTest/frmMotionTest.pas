{ Test for Opencv motion reconstruction class from webcam with ocvWrapper

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
unit frmMotionTest;

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
    procedure btnStartClick(Sender: TObject);
  private
      last: Integer;
      mhi: Pointer; //  # MHI
      orient: Pointer; //  # orientation
      mask: Pointer; //  # valid orientation mask
      segmask: Pointer; //  # motion segmentation map
      vecrect: PCvvector_Rect;
      buf: array[0..10] of PCvMat_t;
      startsec: TDateTime;
      vecmat: PCvvector_Mat;
      matempty: PCvMat_t;
      imgempty: PCvMat_t;
    { Private declarations }
    procedure update_mhi(const img: PCvMat_t; const dst: PCvMat_t; diff_threshold: Integer);
  public
    { Public declarations }
  end;

var
  Form6: TForm6;

implementation

{$R *.lfm}
uses DateUtils;
const
  window_name = 'Motion test';



procedure TForm6.update_mhi(const img: PCvMat_t; const dst: PCvMat_t; diff_threshold: Integer);
const
  N = 4;
  MHI_DURATION = 1.0;
  MAX_TIME_DELTA = 1.5 ;
  MIN_TIME_DELTA = 0.05 ;
  MIN_PERIMETER = 120;
var
  i: Integer;
  timestamp: Single;
  idx1, idx2: Integer;
  silh: PCvMat_t;
  w, h: Integer;
  wm, hm: Integer;
  nrect: Integer;
  prect: PCvRect_t;
  boundrect: CvRectS;

  silhRoi: PCvMat_t;
  mhiRoi: PCvMat_t;
  orientRoi: PCvMat_t;
  maskRoi: PCvMat_t;

  angle: Single;
  globorient: Single;
  count: Double;
  magnitude: Single;
  center: PCvPoint_t;
  pcolor: PCvScalar_t;
  pt1: PCvPoint_t;
begin
//    # get current time in seconds
      timestamp:=SecondsBetween(Time(), startsec);
//     # get current frame size
      w:=pCvMatGetWidth(img);
      h:=pCvMatGetHeight(img);
      if (mhi <> nil) then
      begin
          wm:=pCvMatGetWidth(mhi);
          hm:=pCvMatGetHeight(mhi);
      end;
      idx1 := last;
      if (vecrect = nil) then
      begin
        vecrect:=pCvVectorRectCreate(0);
      end;
      if (mhi = nil) or  (w <> wm) or (h <> hm) then
      begin
        for i:=0 to 9 do
        begin
          buf[i]:=pCvMatImageCreate(w,h,CV_8UC1);
        end;
        mhi:=pCvMatImageCreate(w,h,CV_32FC1);
        orient:=pCvMatImageCreate(w,h,CV_32FC1);
        segmask:=pCvMatImageCreate(w,h,CV_32FC1);

        mask:=pCvMatImageCreate(w,h,CV_8UC1);
        imgempty:=pCvMatImageCreate(w,h, CV_8UC1);
        pCvVectorMatSet(vecmat,1, imgempty);
        pCvVectorMatSet(vecmat,2, imgempty);
        pCvVectorMatSet(vecmat,3, imgempty);
      end;

//     # convert frame to grayscale
      pCvcvtColor(img, buf[last],  COLOR_BGR2GRAY, 0);


//     # index of (last - (N-1))th frame
      idx2:= (last + 1) mod N;
      last := idx2;
      silh:= buf[idx2];

//     # get difference between frames
      pCvabsdiff(buf[idx1], buf[idx2], silh);
//     # and threshold it
      pCvthreshold(silh, silh, diff_threshold, 1, CV_THRESH_BINARY);
//     # update MHI
      pCvupdateMotionHistory(silh, mhi, timestamp, MHI_DURATION);
      pCvconvertScaleAbs(mhi, mask, 255.0/MHI_DURATION, (MHI_DURATION - timestamp)*255./MHI_DURATION);

      pCvVectorMatSet(vecmat, 0, mask);
      pCvmerge(vecmat, dst);
      pCvcalcMotionGradient(mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3);

      pCvsegmentMotion(mhi, segmask, vecrect, timestamp, MAX_TIME_DELTA);

      pcolor:=CvScalar_(0,255,0,0);
      nrect:=pCvVectorRectLength(vecrect);
      for i:=0 to nrect-1 do
      begin
        prect:=pCvVectorRectGet(vecrect, i);
        pCvRectToStruct(prect, @boundrect);

        if (boundrect.width + boundrect.height)>MIN_PERIMETER then   // # reject very small components
        begin
          silhRoi:=pCvMatROI(silh, @boundrect);
          mhiRoi:=pCvMatROI(mhi, @boundrect);
          orientRoi:=pCvMatROI(orient, @boundrect);
          maskRoi:=pCvMatROI(mask, @boundrect);
          globorient:=  pCvcalcGlobalOrientation(orientRoi, maskRoi, mhiRoi, timestamp, MHI_DURATION);
          angle:=360 - globorient;
          count:= pCvnorm(silhRoi, NORM_L1, matempty);  //# calculate number of points within silhouette ROI
          if (count >= (boundrect.width*boundrect.height * 0.05)) then
          begin
            magnitude:=30;
            center:= CvPoint_(boundrect.x + (boundrect.width shr 1), boundrect.y + (boundrect.height shr 1));
            pCvcircle(dst, center, Round(magnitude * 1.2), pcolor, 3, 8, 0);

            pt1:=CvPoint_( Round(boundrect.x + (boundrect.width shr 1) + magnitude*cos(angle * Pi / 180 )),
                           Round(boundrect.y + (boundrect.height shr 1) - magnitude * Sin(angle * Pi / 180 )   )  );

            pCvline(dst, center,  pt1,  pcolor, 3, 8, 0);
            pCvPointDelete(center);
            pCvPointDelete(pt1);
          end;



          pCvMatDelete(silhRoi);
          pCvMatDelete(mhiRoi);
          pCvMatDelete(orientRoi);
          pCvMatDelete(maskRoi);
        end;


        pCvRectDelete(prect);
      end;
      pCvScalarDelete(pcolor);
end;

procedure TForm6.btnStartClick(Sender: TObject);
var
    i: Integer;
    winame: CvString_t;
    capture: PCvVideoCapture_t;
    c: Integer;
    frame: PCvMat_t;
    framegray: PCvMat_t;
    motion: PCvMat_t;

    w,h: Integer;
begin
try
  btnStart.Enabled:=False;
  motion:=nil;
  frame:=nil;
  winame.pstr:= PAnsiChar(AnsiString(window_name));

  capture:=pCvVideoCaptureCreate();
  frame:=pCvMatCreateEmpty();
  matempty:=pCvMatCreateEmpty();

  vecmat:=pCvVectorMatCreate(4);

  last:=0;
  mhi:=nil;
  orient:=nil;
  mask:=nil;
  segmask:=nil;
  vecrect:=nil;
  startsec:=Time();


  pCvVideoCaptureopen2(capture, 0);
  if not(pCvVideoCaptureisOpened(capture)) then Exit;
  while True do
  begin
     if not(pCvVideoCaptureread(capture, frame)) then Break;

     if (motion = nil) then
     begin
        w:=pCvMatGetWidth(frame);
        h:=pCvMatGetHeight(frame);
        motion:=pCvMatImageCreate(w, h, CV_8UC3);
     end;
     update_mhi(frame, motion, 30);

     pCvimshow(@winame, motion );
     c:=pCvwaitKey(10);
     if Char(c) = 'c' then Break;
  end;
  pCvdestroyWindow(@winame);
  btnStart.Enabled:=True;
finally
  pCvVideoCaptureDelete(capture);
  pCvMatDelete(frame);
  pCvMatDelete(matempty);
  pCvMatDelete(imgempty);
  pCvVectorMatDelete(vecmat);

  pCvMatDelete(motion);
  pCvMatDelete(mhi);
  pCvMatDelete(orient);
  pCvMatDelete(segmask);
  pCvMatDelete(mask);
  pCvVectorRectDelete(vecrect);

  for i:=0 to 9 do
  begin
    pCvMatDelete(buf[i]);
  end;
end;

end;

end.
