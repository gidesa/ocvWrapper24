{ ocvWrapper: wrapper for C++ API  Opencv interface 

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
unit OPENCVWrapper;
{$IFDEF FPC}
  {$MODE Delphi}
{$ENDIF}

interface

const

{$INCLUDE 'unOcvWrapper_const.pas'}

type
__intern__1 = record  end;
__intern__2 = record  end;
__intern__3 = record  end;
__intern__4 = record  end;
__intern__5 = record  end;
__intern__6 = record  end;
__intern__7 = record  end;
__intern__8 = record  end;
__intern__9 = record  end;
__intern__10 = record  end;
__intern__11 = record  end;
__intern__12 = record  end;
__intern__13 = record  end;
__intern__14 = record  end;
__intern__15 = record  end;
__intern__16 = record  end;
__intern__17 = record  end;
__intern__18 = record  end;
__intern__19 = record  end;
__intern__20 = record  end;
__intern__21 = record  end;
__intern__22 = record  end;
__intern__23 = record  end;

__intern__24 = record  end;
__intern__25 = record  end;
__intern__26 = record  end;
__intern__27 = record  end;
__intern__28 = record  end;
__intern__29 = record  end;
__intern__30 = record  end;
__intern__31 = record  end;
__intern__32 = record  end;
__intern__33 = record  end;
__intern__34 = record  end;
__intern__35 = record  end;
__intern__36 = record  end;
__intern__37 = record  end;
__intern__38 = record  end;
__intern__39 = record  end;

__intern__40 = record  end;
__intern__41 = record  end;
__intern__42 = record  end;
__intern__43 = record  end;
__intern__44 = record  end;
__intern__45 = record  end;
__intern__46 = record  end;
__intern__47 = record  end;
__intern__48 = record  end;

  { IplImage structure from old Opencv C API }
  TIplROI = record
    Coi     : Integer;
    XOffset : Integer;
    YOffset : Integer;
    Width   : Integer;
    Height  : Integer;
  end;
  PIplROI = ^TIplROI;

  PIplImage = ^TIplImage;
  TIplImage = record
    NSize           : Integer;                 // size of iplImage struct
    ID              : Integer;                 // version
    NChannels       : Integer;
    AlphaChannel    : Integer;
    Depth           : Integer;                 // pixel depth in bits
    ColorModel      : array [0..3] of AnsiChar;
    ChannelSeq      : array [0..3] of AnsiChar;
    DataOrder       : Integer;
    Origin          : Integer;
    Align           : Integer;                 // 4 or 8 byte align
    Width           : Integer;
    Height          : Integer;
    Roi             : PIplROI;
    MaskROI         : PIplImage;               // poiner to maskROI if any
    ImageId         : Pointer;                 // use of the application
    TileInfo        : Pointer;            // contains information on tiling
    ImageSize       : Integer;                 // useful size in bytes
    ImageData       : PByte;                   // pointer to aligned image
    WidthStep       : Integer;                 // size of aligned line in bytes
    BorderMode      : array [0..3] of Integer;
    BorderConst     : array [0..3] of Integer;
    ImageDataOrigin : PByte;                   // ptr to full, nonaligned image
  end;


{ Opencv and C++ native classes, vectors, pointers type }
  PCvMat_t = ^__intern__1;
  PCvVec2d_t = ^__intern__3;
  PCvVec3d_t = ^__intern__4;
  PCvVec3b_t = ^__intern__5;
  PCvVec4f_t = ^__intern__6;
  PCvVec6f_t = ^__intern__7;
  PCvSize_t = ^__intern__8;
  PCvRect_t = ^__intern__9;
  PCvRotatedRect_t = ^__intern__10;
  PCvPoint_t = ^__intern__11;
  PCvPoint2d_t = ^__intern__12;
  PCvPoint2f_t = ^__intern__13;
  PCvScalar_t = ^__intern__14;
  PCvTermCriteria_t = ^__intern__15;
  PCvCvTermCriteria_t = ^__intern__16;
  PCvRange_t = ^__intern__17;
  PCvCvSlice_t = ^__intern__18;
  PCvCvDTreeNode_t = ^__intern__19;
  PCvMoments_t = ^__intern__20;
  PCvIndexParams_t = ^__intern__21;
  PCvSearchParams_t = ^__intern__22;

  PCvvector_uchar = ^__intern__23;
  PCvvector_int = ^__intern__24;
  PCvvector_float = ^__intern__25;
  PCvvector_double = ^__intern__26;
  PCvvector_Point = ^__intern__27;
  PCvvector_Point2f = ^__intern__28;
  PCvvector_Vec4f = ^__intern__29;
  PCvvector_Vec6f = ^__intern__30;
  PCvvector_Rect = ^__intern__31;
  PCvvector_KeyPoint = ^__intern__32;
  PCvvector_Mat = ^__intern__33;
  PCvvector_DMatch = ^__intern__34;
  PCvvector_string = ^__intern__35;
  PCvvector_vector_Point = ^__intern__36;
  PCvvector_vector_Point2f = ^__intern__37;
  PCvvector_vector_DMatch = ^__intern__38;


  PCvPtr_Algorithm = ^__intern__39;
  PCvPtr_Feature2D = ^__intern__40;
  PCvPtr_FaceRecognizer = ^__intern__41;
  PCvPtr_CLAHE = ^__intern__42;
  PCvPtr_FeatureDetector = ^__intern__43;
  PCvPtr_DescriptorExtractor = ^__intern__44;
  PCvPtr_DescriptorMatcher = ^__intern__45;
  PCvPtr_flann_IndexParams = ^__intern__46;
  PCvPtr_flann_SearchParams = ^__intern__47;



{ helper records for some simple Opencv classes }
  CvString_t   = record
      pstr: PAnsiChar;
      nrchar: Integer;
  end;
  PCvString_t = ^CvString_t;


  CvPoint2fS = record
    x: Single;
    y: Single;
  end {CvPoint2fS};

  CvPoint2dS = record
    x: Double;
    y: Double;
  end {CvPoint2dS};

  CvPointS = record
    x: Integer;
    y: Integer;
  end {CvPointS};

  CvSizeS = record
    width: Integer;
    height: Integer;
  end {CvSizeS};

  CvRectS = record
    x: Integer;
    y: Integer;
    width: Integer;
    height: Integer;
  end {CvRectS};

  CvRangeS = record
    _start: Integer;
    _end: Integer;
  end {CvRangeS};

  CvSliceS = record
    _start: Integer;
    _end: Integer;
  end {CvSliceS};

  CvVec2dS = record
    v0: Double;
    v1: Double;
  end {CvVec2dS};

  CvVec3dS = record
    v0: Double;
    v1: Double;
    v2: Double;
  end {CvVec3dS};


  CvVec3bS = record
    v0: byte;
    v1: byte;
    v2: byte;
  end {CvVec3bS};


  CvVec4fS = record
    v0: Double;
    v1: Double;
    v2: Double;
    v3: Double;
  end {CvVec4fS};

  CvVec6fS = record
    v0: Double;
    v1: Double;
    v2: Double;
    v3: Double;
    v4: Double;
    v5: Double;
  end {CvVec6fS};

  CvScalarS = record
    v0: Double;
    v1: Double;
    v2: Double;
    v3: Double;
  end {CvScalarS};

  CvRotatedRectS = record
    center: CVPOINT2FS;
    size: CVSIZES;
    angle: Single;
  end {CvRotatedRectS};

  CvFileNodeS = record
    tag: Integer;
    info: Pointer;
{-  information }
{=(only for user-defined object, for others it is 0) }
    f: Double;
{= scalar floating-point number }
    i: Integer;
{= scalar integer number }
    strlen: Integer;
{= text string }
    str: PChar;
    seq: Pointer;
{= sequence (ordered collection of file nodes) }
    map: Pointer;
{= map (collection of named file nodes) }
  end {CvFileNodeS};


  CvTermCriteriaS = record
    _type: Integer;
{- may be combination of }
{-CV_TERMCRIT_ITER }
{=CV_TERMCRIT_EPS }
    max_iter: Integer;
    epsilon: Double;
  end {CvTermCriteriaS};

  CvDTreeNodeS = record
    class_idx: Integer;
    Tn: Integer;
    value: Double;
    parent: Pointer;
    left: Pointer;
    right: Pointer;
    split: Pointer;
    sample_count: Integer;
    depth: Integer;
    num_valid: PInteger;
    offset: Integer;
    buf_idx: Integer;
    maxlr: Double;
{/// global pruning data }
    complexity: Integer;
    alpha: Double;
    node_risk, tree_risk, tree_error: Double;
{/// cross-validation pruning data }
    cv_Tn: PInteger;
    cv_node_risk: PDouble;
    cv_node_error: PDouble;
  end {CvDTreeNodeS};

  CvMomentsS = record
    m00, m10, m01, m20, m11, m30, m21, m12, m03: Double;
{= spatial moments }
    mu20, mu11, mu02, mu30, mu21, mu12, mu03: Double;
{= central moments }
    nu20, nu11, nu02, nu30, nu21, nu12, nu03: Double;
  end {CvMomentsS};

PCvPoint2fS = ^CvPoint2fS;
PCvPoint2dS = ^CvPoint2dS;
PCvPointS = ^CvPointS;
PCvSizeS = ^CvSizeS;
PCvRectS = ^CvRectS;
PCvRangeS = ^CvRangeS;
PCvSliceS = ^CvSliceS;
PCvVec2dS = ^CvVec2dS;
PCvVec3bS = ^CvVec3bS;
PCvVec3dS = ^CvVec3dS;
PCvVec4fS = ^CvVec4fS;
PCvVec6fS = ^CvVec6fS;
PCvScalarS = ^CvScalarS;
PCvRotatedRectS = ^CvRotatedRectS;
PCvFileNodeS = ^CvFileNodeS;
PCvTermCriteriaS = ^CvTermCriteriaS;
PCvDtreeNodeS = ^CvDtreeNodeS;
PCvMomentsS = ^CvMomentsS;


{$INCLUDE   'unOcvWrapper_types.pas'}
{------------- Pascal helpers ---------------}
  function CvSize_(width, height: Integer; pcvsize: PCvSize_t = nil): PCvSize_t;
  function CvScalar_(v0, v1, v2, v3: Double; pcvscalar: PCvScalar_t = nil): PCvScalar_t;
  function CvPoint_(x, y: Integer; pcvpoint: PCvPoint_t = nil): PCvPoint_t;
  function CvVec3b_(v0, v1, v2: byte; pcvvec3b: PCvVec3b_t = nil): PCvVec3b_t;
  function CvTermCriteria_(tcType: integer; max_iter: Integer; epsilon: Double;
                           pcvtermcrit: PCvCvTermCriteria_t = nil): PCvCvTermCriteria_t;
{--------------------------------------------}
{ --------------- User functions ------------}
  function   pCvRedirectException(const func: Pointer): Boolean; cdecl;
  procedure  pCvDrawMatches(const img1: PCvMat_t; const keypoints1: PCvvector_KeyPoint;
            const img2: PCvMat_t; const keypoints2: PCvvector_KeyPoint;
            const matches1to2: PCvvector_DMatch; outImg: PCvMat_t;
            const matchColor: PCvScalar_t; const singlePointColor: PCvScalar_t ); cdecl;
  procedure  pCvPCACompute2(data: PCvMat_t; mean: PCvMat_t; eigenvectors: PCvMat_t;
                           eigenvalues: PCvMat_t;  maxComponents: Integer { default: 0 }); cdecl;

{ C++ string class }
  function   pCvStringCreate(const nrchar: Integer): PCvString_t;   cdecl;
  procedure  pCvStringDelete(const cvstr: PCvString_t);  cdecl;

{ Opencv Mat class }
  function   pCvMatCreate(const ndims: Integer; const dims: PInteger; const mtype: integer): PCvMat_t; cdecl;
  function   pCvMatCreateEmpty (): PCvMat_t  ; cdecl;
  function   pCvMat2dCreate (const rows: Integer; const cols: Integer; mtype: integer): PCvMat_t ; cdecl;
  function   pCvMatImageCreate(const width: Integer; height: Integer; mtype: Integer): PCvMat_t; cdecl;
  procedure  pCvMatDelete (const mat: PCvMat_t); cdecl;
  function   pCvMatROI(const mat: PCvMat_t; const roi: PCvRectS): PCvMat_t; cdecl;
  procedure  pCvMatFill(const wrapper: PCvMat_t; const virtual: PCvScalar_t); cdecl;
  procedure  pCvMatCopy(const src: PCvMat_t; const dst: PCvMat_t); cdecl;
  function   pCvMatSetByte(const mat: PCvMat_t; const rowind: Integer; const colind: Integer; const val: Byte): Byte; cdecl;
  function   pCvMatGetByte(const mat: PCvMat_t; const rowind: Integer; const colind: Integer): Byte; cdecl;
  function   pCvMatSetInt(const mat: PCvMat_t; const rowind: Integer; const colind: Integer; const val: Integer): Integer; cdecl;
  function   pCvMatGetInt(const mat: PCvMat_t; const rowind: Integer; const colind: Integer): Integer; cdecl;
  function   pCvMatSetFloat(const mat: PCvMat_t; const rowind: Integer; const colind: Integer; const val: Single): Single; cdecl;
  function   pCvMatGetFloat(const mat: PCvMat_t; const rowind: Integer; const colind: Integer): Single; cdecl;
  function   pCvMatSetFloatMultidim(const mat: PCvMat_t; const indexes: PInteger; const val: Single): Single; cdecl;
  function   pCvMatGetFloatMultidim(const mat: PCvMat_t; const indexes: PInteger; const colind: Integer): Single; cdecl;
  function   pCvMatSetDouble(const mat: PCvMat_t; const rowind: Integer; const colind: Integer; const val: Double): Double; cdecl;
  function   pCvMatGetDouble(const mat: PCvMat_t; const rowind: Integer; const colind: Integer): Double; cdecl;
  function   pCvMatSetPixelC3(const mat: PCvMat_t; const rowind: Integer; const colind: Integer; const val: PCvVec3b_t ): PCvVec3b_t; cdecl;
  function   pCvMatGetPixelC3(const mat: PCvMat_t; const rowind: Integer; const colind: Integer): PCvVec3b_t; cdecl;
  function   pCvMatGetWidth(const mat: PCvMat_t): Integer ; cdecl;
  function   pCvMatGetHeight(const mat: PCvMat_t): Integer ; cdecl;
  function   pCvMatGetChannels(const mat: PCvMat_t): Integer ; cdecl;
  function   pCvMatGetType(const mat: PCvMat_t): Integer ; cdecl;
  function   pCvMatGetDims(const mat: PCvMat_t): Integer ; cdecl;
  function   pCvMatGetData(const mat: PCvMat_t): Pointer ; cdecl;
  function   pCvMatGetDepth(const mat: PCvMat_t): Integer ; cdecl;

  function   pCvMatToIplImage(const mat: PCvMat_t): PIplImage; cdecl;
  procedure  pCvIplImageToMat(const iplim: PIplImage; mat: PCvMat_t); cdecl;
  procedure  pCvIplImageDelete (const iplimg: PIplImage ); cdecl;

{----- Other Opencv and C++ native classes -----------}
{$INCLUDE   'unOcvWrapper_nativeClasses.pas'}
{----- Opencv classes and global fnctions  -----------}
{$INCLUDE   'unOcvWrapper_functions.pas'}

implementation

 {$IFDEF FPC}
 uses SysUtils;
 {$ELSE}
 uses System.SysUtils;
 {$ENDIF}
 const
 {$IFDEF DEBUG}
 ocvWrapper = 'D:\GDS\progC\ocvWrapper\bin\Debug\x86\ocvCPPWrapper24.dll';
 {$ELSE}
 ocvWrapper = 'ocvCPPWrapper24.dll';
 {$ENDIF}
 function   pCvRedirectException;         external ocvWrapper name 'pCvRedirectException';
 procedure  pCvDrawMatches;               external ocvWrapper name 'pCvDrawMatches';
 procedure  pCvPCACompute2;               external ocvWrapper name 'pCvPCACompute2';

 function   pCvStringCreate;              external ocvWrapper name 'pCvStringCreate';
 procedure  pCvStringDelete;              external ocvWrapper name 'pCvStringDelete';

 function   pCvMatCreate;                 external ocvWrapper name 'pCvMatCreate';
 function   pCvMat2dCreate;               external ocvWrapper name 'pCvMat2dCreate';
 function   pCvMatCreateEmpty;            external ocvWrapper name 'pCvMatCreateEmpty';
 function   pCvMatImageCreate;            external ocvWrapper name 'pCvMatImageCreate';
 procedure  pCvMatDelete;                 external ocvWrapper name 'pCvMatDelete';
 function   pCvMatROI;                    external ocvWrapper name 'pCvMatROI';
 procedure  pCvMatCopy;                   external ocvWrapper name 'pCvMatCopy';
 procedure  pCvMatFill;                   external ocvWrapper name 'pCvMatFill';
 function   pCvMatGetByte;                external ocvWrapper name 'pCvMatGetByte';
 function   pCvMatSetByte;                external ocvWrapper name 'pCvMatSetByte';
 function   pCvMatGetInt;                 external ocvWrapper name 'pCvMatGetInt';
 function   pCvMatSetInt;                 external ocvWrapper name 'pCvMatSetInt';
 function   pCvMatGetFloat;               external ocvWrapper name 'pCvMatGetFloat';
 function   pCvMatSetFloat;               external ocvWrapper name 'pCvMatSetFloat';
 function   pCvMatGetFloatMultidim;       external ocvWrapper name 'pCvMatGetFloatMultidim';
 function   pCvMatSetFloatMultidim;       external ocvWrapper name 'pCvMatSetFloatMultidim';
 function   pCvMatGetDouble;              external ocvWrapper name 'pCvMatGetDouble';
 function   pCvMatSetDouble;              external ocvWrapper name 'pCvMatSetDouble';
 function   pCvMatGetPixelC3;             external ocvWrapper name 'pCvMatGetPixelC3';
 function   pCvMatSetPixelC3;             external ocvWrapper name 'pCvMatSetPixelC3';
 function   pCvMatGetWidth;               external ocvWrapper name 'pCvMatGetWidth';
 function   pCvMatGetHeight;              external ocvWrapper name 'pCvMatGetHeight';
 function   pCvMatGetChannels;            external ocvWrapper name 'pCvMatGetChannels';
 function   pCvMatGetType;                external ocvWrapper name 'pCvMatGetType';
 function   pCvMatGetDims;                external ocvWrapper name 'pCvMatGetDims';
 function   pCvMatGetData;                external ocvWrapper name 'pCvMatGetData';
 function   pCvMatGetDepth;               external ocvWrapper name 'pCvMatGetDepth';
 procedure  pCvIplImageToMat;             external ocvWrapper name 'pCvIplImageToMat';
 function   pCvMatToIplImage;             external ocvWrapper name 'pCvMatToIplimage';
 procedure  pCvIplImageDelete;            external ocvWrapper name 'pCvIplImageDelete';


{$INCLUDE 'unOcvWrapper_nativeClasses_extern.pas'}
{$INCLUDE 'unOcvWrapper_extern.pas'}


procedure cvException(msg: PCvString_t);
begin
   raise Exception.Create('OpenCV Error: '+msg^.pstr);
end;


function CvSize_(width, height: Integer; pcvsize: PCvSize_t): PCvSize_t;
var
  csize: CvSizeS;
begin
  csize.width:=width;
  csize.height:=height;
  if pcvsize = nil then
        Result:=pCvSizeCreate()
  else
     result:=pcvsize;
  pCvSizeFromStruct(Result, @csize);
end;

function CvScalar_(v0, v1, v2, v3: Double; pcvscalar: PCvScalar_t): PCvScalar_t;
var
  cscalar: CvScalarS;
begin
  cscalar.v0 :=v0;
  cscalar.v1 :=v1;
  cscalar.v2 :=v2;
  cscalar.v3 :=v3;
  if pcvscalar = nil then
    Result := pCvScalarCreate()
  else
    Result := pcvscalar;

  pCvScalarFromStruct(Result, @cscalar);
end;

function CvPoint_(x, y: Integer; pcvpoint: PCvPoint_t): PCvPoint_t;
var
  cpoint: CvPointS;
begin
  cpoint.x := x;
  cpoint.y := y;
  if pcvpoint = nil then
    Result := pCvPointCreate()
  else
    Result := pcvpoint;

  pCvPointFromStruct(Result, @cpoint);
end;

function CvVec3b_(v0, v1, v2: byte; pcvvec3b: PCvVec3b_t): PCvVec3b_t;
var
  cvec3b: Cvvec3bS;
begin
  cvec3b.v0 :=v0;
  cvec3b.v1 :=v1;
  cvec3b.v2 :=v2;
  if pcvvec3b = nil then
    Result := pCvvec3bCreate()
  else
    Result := pcvvec3b;

  pCvvec3bFromStruct(Result, @cvec3b);
end;

function CvTermCriteria_(tcType: integer; max_iter: Integer; epsilon: Double;
                   pcvtermcrit: PCvCvTermCriteria_t): PCvCvTermCriteria_t;
var
  ctermcrit: CvTermCriteriaS;
begin
  ctermcrit._type:=tcType;
  ctermcrit.max_iter:=max_iter;
  ctermcrit.epsilon:=epsilon;
  if pcvtermcrit = nil then
    Result:=pCvCvTermCriteriaCreate()
  else
    Result:=pcvtermcrit;

  pCvCvTermCriteriaFromStruct(Result, @ctermcrit);
end;
{****************************************************************************}
begin
    pCvRedirectException(@cvexception);
end.
