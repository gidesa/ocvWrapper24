# ocvWrapper24 v 0.9
Opencv  C and Delphi wrapper for C++ classes

This project contains a dynamic library (DLL in Windows) that is a wrapper
around the C++ API of Opencv v 2.4.13 .
The library expose a pure C API interface to all C++ classes and funtions of Opencv.
More than 1.100 C wrapper functions cover vast majority of Opencv C++ classes and global
functions, and some native C++ classes.
The library is accessed by Delphi units with signatures to all the wrapper C API.

# Bases

The wrapper code is based on the Python Opencv interface. So, all Opencv classes or functions
exposed to Python are also exposed in wrapper.
The code is in general divided in two parts:
- "generated" code, so called because it's automatically and totally generated from scripts using some annotations
included in Opencv C++ headers (see https://docs.opencv.org/3.0.0/da/d49/tutorial_py_bindings_basics.html). This code must not be modified manually .
- "native" code, so called because contains base Opencv classes and also native C++ classes that are not generated
from script, but are written manually.

# Types of classes in Opencv

- standard Opencv classes: these have a standard constructor/destructor, properties with get and set, and methods.
- pointer to classes (Ptr): these pointers are returned from some "factory" global functions. So these classes
  haven't a regular constructor, and the pointer require a different treatment from standard class reference.
- Opencv structs: there are also some structs used among the C++ classes, these structs, in wrapper, for simplicity
  are treated as the standard classes, so have a pseudo constructor/destructor.
- C++ vectors: Opencv sometimes use C++ vectors, and vectors of vectors. In wrapper these are treated as standard
  classes, of course with some specific methods
  
# Wrapper implementation

Wrapper code bind a special C struct to every class, Opencv struct, Ptr or vector. The binding struct end with \"_t\" suffix.
The struct contains internally a field with the reference to the real Opencv internal entity (class, struct, ecc.).

Every Opencv/C++ class is "flattened" to a group of C functions: one for create, one for delete, one for every method, one for
every property get, and one for every property set. The create function returns the binding struct, and the delete function release the memory
allocated for the entity and for the struct iself. 

Only Ptr, as said, haven't the create function but only delete function.
When the C or Delphi caller program  invoke some method or function, it pass always as the first parameter the pointer to binding
struct, that was returned from the corresponding create function.

All C wrapper functions have the "pCv" prefix, after that the class name, after that the original Opencv method name:

  pCv\<class name\>\<method name\>
  
Properties get and set have the standard name schema:
  
  pCv\<class name\>Set_\<property name\>
  
  pCv\<class name\>Get_\<property name\>
  
Example: the standard Mat Opencv class has the binding struct called Mat_t, a constructor called pCvMatCreate 
(indeed plus many others constructors because it's a special class), and a destructor called pCvMatDelete.
  
At last, for some very simple classes or structs, with few fields, there aren't single get/set functions for every property,
but instead only two functions FromStruct and ToStruct that get or set all members from an helper C struct with a single call. So:
  
  pCv\<class name\>FromStruct
  
  pCv\<class name\>ToStruct
  
Ptr types have another specialized function that convert from Ptr entity to the standard wrapper binding struct. This because
in general all wrapper functions "understand" only the binding struct format, not Ptr.
  
For example, for Ptr_Feature2d type there is a pCvPtr_Feature2dConvert function,  that has input parameter type Ptr_feature2d and
returns a pointer to a fresh Feature2d_t binding struct. When calling a method of Feature2D class (example: pCvFeature2dcompute) 
you will use the binding struct as first parameter, not the original Ptr.
  
# Installation (Windows)

Download Opencv version 2.4.13.6 from https://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.13/opencv-2.4.13.6-vc14.exe/download

Install Opencv, and add in Windows path the directory with Opencv DLL (for example: C:\opencv2.4.13.6\build\x86\vc14\bin):

  set path=C:\opencv2.4.13.6\build\x86\vc14\bin;%path%

Download this repository and add in Windows path the directory with ocvWrapper.dll (note: for now there is only the 32 bit binary DLL).
If you have copied repository in C:\ocvWrapper, then:

  set path=C:\ocvWrapper\bin\Release\x86

Now Opencv and ocvWrapper can be used from every program.

# Delphi use

To use ocvWrapper from Delphi only add the main unit OPENCVWrapper.pas to Uses clause
of program.
Other .pas files are included directly from OPENCVWrapper.pas 
In main unit there are many definitions of "fake" records, named as
__inter§xx . They served only as a trick to force the Delphi IDE and compiler to discriminate
the various "opaque" pointers to wrapper structures. 

Without the trick, all that pointers are only .... pointers, not different one from another!
So IDE cannot help in code suggestion, and compiler cannot catch wrong use.




# Example programs

- testKNN: a pure C program that use the K Nearest Neighbors class from Machine Learning module (ML)

- testPCA: a Delphi program for testing the PCA class from ML module

- testSVM: a Delphi program for testing CvSVM class from ML module

- motionTest: a Delphi demo program for motion tracking

- featureTest: a Delphi demo program for ORB feature detector class and matching between images

- cascadeTest: a Delphi demo program for face recognition on webcam, by cascade classifier class

- datamatrixTest: a Delphi demo program fro Datamatrix code recognition on webcam

# Build from sources

The library can be rebuilt using the Visual C++ 2017 project in ocvCPPWrapper24 directory.
Only dependence are the .lib files of Opencv, found in  \build directory inside Opencv
root directory.


