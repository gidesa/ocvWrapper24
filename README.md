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
- "generate" code, so called because it's automatically completely generated using some annotations
included in Opencv C++ headers (see https://docs.opencv.org/3.0.0/da/d49/tutorial_py_bindings_basics.html). This code must not modified manually .
- "native" code, so called because contains base Opencv classes and also native C++ classes that are not generated
from generator script, but are written manually

# Types of classes in Opencv


