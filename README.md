# ocvWrapper24 v 0.9
Opencv  C and Delphi wrapper for C++ classes

This project contains a dynamic library (DLL in Windows) that is a wrapper
around the C++ API of Opencv v 2.4.13 .
The library expose a pure C API interface to all C++ classes and funtions of Opencv.
In turn there are Delphi units to call the various wrapper C API.

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
The struct contains internally a field with the reference to the real Opencv internal entity (class, struct, ecc.)
For every Opencv/C++ type there are a create function, that return the binding struct, and a delete function, that release the memory
allocated for the entity and for the struct iself. Only Ptr, as said, haven't the create function but only delete function.
When the C or Delphi caller program  invoke some method or function, it pass always as the first parameter the pointer to binding
struct, returned from corresponding create function.

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
  

   

