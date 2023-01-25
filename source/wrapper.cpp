/* ocvWrapper: wrapper for C++ API  Opencv interface 

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
*/

#ifdef _USRDLL
#define CVAPI_EXPORTS  1
#else
#undefine  CVAPI_EXPORTS
#endif

#include "opencv2/core/core.hpp"
#include "opencv2/ml/ml.hpp"

#include  "opencv2/flann/miniflann.hpp"
#include  "opencv2/imgproc/imgproc.hpp"
#include  "opencv2/calib3d/calib3d.hpp"
#include  "opencv2/features2d/features2d.hpp"
#include  "opencv2/video/tracking.hpp"
#include  "opencv2/photo/photo.hpp"
#include  "opencv2/video/background_segm.hpp"
#include  "opencv2/objdetect/objdetect.hpp"
#include  "opencv2/contrib/contrib.hpp"
#include  "opencv2/highgui/highgui.hpp"
#include  "opencv2/nonfree/features2d.hpp"
#include  "opencv2/nonfree/nonfree.hpp"

using namespace cv;            // OpenCV API is in the C++ "cv" namespace

#include <cstdio>
using namespace std;

#include "ocvWrp_nativeTypes.h"

string_t lastError;
void(*customException) (string_t*) = NULL;

CVAPI(bool) pCvRedirectException(void * func) {
	if (func != NULL) { customException = (void(*) (string_t*)) (func); return true; }
	else { return false; }
}

void exceptionDisplay(const string msg) {
	if (customException != NULL) {
		string_t* wrapper = new string_t();
		int l = msg.length() + 1;
		wrapper->v = (char*)cvAlloc(l * sizeof(char));
		wrapper->nrchar = l;
		strcpy(wrapper->v, &msg[0]);

		customException(wrapper);
		cvFree_((void*)wrapper->v);
		delete wrapper;
	}
	else {
		cout << msg;
		throw;
	};
}



#include "ocvWrp_c_generated_types.h"
#include "ocvWrp_nativeClasses.h"
#include "ocvWrp_nativePtr.h"
#include "ocvWrp_gen_Types_Vectors.h"
#include "ocvWrp_c_generated_classes.h"
#include "ocvWrp_c_generated_global_funcs.h"
#include "ocvWrp_c_user_functions.h"


