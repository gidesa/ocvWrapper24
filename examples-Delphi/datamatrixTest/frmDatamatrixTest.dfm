object Form6: TForm6
  Left = 0
  Top = 0
  Caption = 'Datamatrix code test'
  ClientHeight = 120
  ClientWidth = 306
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'Tahoma'
  Font.Style = []
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 13
  object lbl1: TLabel
    Left = 32
    Top = 80
    Width = 241
    Height = 32
    AutoSize = False
    Caption = 
      'Library can recognize Datamatrix codes of length 3 alphabetic ch' +
      'aracters, not numbers'#13#10
    WordWrap = True
  end
  object btnStart: TButton
    Left = 88
    Top = 32
    Width = 121
    Height = 25
    Caption = 'Start webcam capture'
    TabOrder = 0
    OnClick = btnStartClick
  end
end
