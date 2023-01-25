{ Test for Opencv  SVM class with ocvWrapper

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
}unit frmTestSVM;

interface

uses
  Winapi.Windows, Winapi.Messages, System.SysUtils, System.Variants, System.Classes, Vcl.Graphics,
  Vcl.Controls, Vcl.Forms, Vcl.Dialogs,
  OPENCVWrapper, Vcl.StdCtrls;

type
  TForm7 = class(TForm)
    btn1: TButton;
    procedure btn1Click(Sender: TObject);
  private
    { Private declarations }                    public
    { Public declarations }
  end;

var
  Form7: TForm7;

implementation

{$R *.dfm}


procedure TForm7.btn1Click(Sender: TObject);
var
  i, j: Integer;
  width, height: Integer;
  response: Single;
  ret: Boolean;
  thickness: Integer;
  linetype: Integer;
  cstr: CvString_t;

  image: PCvMat_t;
  trainingDataMat: PCvMat_t;
  labelsMat: PCvMat_t;
  matempty: PCvMat_t;
  sampleMat: PCvMat_t;
  svmclassifier: PCvCvSVM_t;
  termcrit: PCvCvTermCriteria_t;
  svmparams: PCvCvSVMParams_t;
  colgreen, colblue: PCvVec3b_t;
  col1, col2: PCvScalar_t;
  cent: PCvPoint_t;
begin
    // Data for visual representation
      width := 512; height := 512;
      image :=  pCvMat2dCreate(width, height, CV_8UC3);
    // Set up training data
      trainingDataMat:=pCvMat2dCreate(4, 2, CV_32FC1);
      pCvMatSetFloat(trainingDataMat, 0, 0, 501); pCvMatSetFloat(trainingDataMat, 0, 1, 10);
      pCvMatSetFloat(trainingDataMat, 1, 0, 255); pCvMatSetFloat(trainingDataMat, 1, 1, 10);
      pCvMatSetFloat(trainingDataMat, 2, 0, 501); pCvMatSetFloat(trainingDataMat, 2, 1, 255);
      pCvMatSetFloat(trainingDataMat, 3, 0, 10) ; pCvMatSetFloat(trainingDataMat, 3, 1, 501);

      labelsMat:=pCvMat2dCreate(4, 1, CV_32SC1);
      pCvMatSetInt(labelsMat, 0, 0, +1);
      for i:=1 to 3 do
      begin
        pCvMatSetInt(labelsMat, i, 0, -1);
      end;
      matempty:=pCvMatCreateEmpty();
      svmparams:=pCvCvSVMParamsCreate();

    // Train the SVM
      svmclassifier:= pCvCvSVMCreate();
      pCvCvSVMParamsSet_svm_type(svmparams, SVM_C_SVC);
      pCvCvSVMParamsSet_kernel_type(svmparams, SVM_LINEAR);
      termcrit:=CvTermCriteria_(TERMCRITERIA_MAX_ITER, 100, 0.000001);
      pCvCvSVMParamsSet_term_crit(svmparams, termcrit);

      ret:= pCvCvSVMtrain(svmclassifier, trainingDataMat, labelsMat, matempty, matempty, svmparams);

    // Show the decision regions given by the SVM
      colgreen:=CvVec3b_(0,255,0);
      colblue:=CvVec3b_(255,0,0);
      sampleMat:=pCvMat2dCreate(1,2, CV_32FC1);
      for i:=0 to  height-1 do
          for j:=0 to  width-1 do
          begin
            pCvMatSetFloat(sampleMat, 0, 0, j);
            pCvMatSetFloat(sampleMat, 0, 1, i);
            response := pCvCvSVMpredict(svmclassifier, sampleMat,False);
            if response=1 then
               pCvMatSetPixelC3(image, i, j, colgreen)
            else
              if response=-1 then
                 pCvMatSetPixelC3(image, i, j, colblue)
          end;
    // Show the training data
     thickness := -1;
     lineType := 8;
     col1:=CvScalar_(0,0,0,0);
     col2:=CvScalar_(255,255,255,255);
     cent:=CvPoint_(0,0);
     pCvcircle( image, CvPoint_(501,  10, cent), 5, col1, thickness, lineType, 0 );
     pCvcircle( image, CvPoint_(255,  10, cent), 5, col2, thickness, lineType, 0 );
     pCvcircle( image, CvPoint_(501, 255, cent), 5, col2, thickness, lineType, 0 );
     pCvcircle( image, CvPoint_( 10, 501, cent), 5, col2, thickness, lineType, 0 );

     cstr.pstr:=PAnsiChar(AnsiString('SVM Simple Example'));
     pCvimshow(@cstr, image); // show it to the user

    // Save the SVM data
     pCvCvSVMsave(svmclassifier, PAnsiChar('svm-data.xml'), PAnsiChar('testSVM'));

     pCvMatDelete(image);
     pCvMatDelete(trainingDataMat);
     pCvMatDelete(labelsMat);
     pCvMatDelete(matempty);
     pCvMatDelete(sampleMat);
     pCvCvSVMParamsDelete(svmparams);
     pCvCvSVMDelete(svmclassifier);
     pCvCvTermCriteriaDelete(termcrit);
     pCvVec3bDelete(colgreen);
     pCvVec3bDelete(colblue);
     pCvScalarDelete(col1);
     pCvScalarDelete(col2);
     pCvPointDelete(cent);


end;

end.
