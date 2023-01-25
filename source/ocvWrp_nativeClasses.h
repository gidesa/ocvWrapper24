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
#pragma once

template <typename T>
static cv::Ptr<T> *Ptr_cpy(const cv::Ptr<T> &ptr)
{
	return new cv::Ptr<T>(ptr);
}

void  Moments_cpy(Moments_t* dest, const  Moments src) {
	dest->v->m00 = src.m00;
	dest->v->m01 = src.m01;
	dest->v->m02 = src.m02;
	dest->v->m03 = src.m03;
	dest->v->m10 = src.m10;
	dest->v->m11 = src.m11;
	dest->v->m12 = src.m12;
	dest->v->m20 = src.m20;
	dest->v->m21 = src.m21;
	dest->v->m30 = src.m30;
	dest->v->mu02 = src.mu02;
	dest->v->mu03 = src.mu03;
	dest->v->mu11 = src.mu11;
	dest->v->mu12 = src.mu12;
	dest->v->mu20 = src.mu20;
	dest->v->mu21 = src.mu21;
	dest->v->mu30 = src.mu30;
	dest->v->nu02 = src.nu02;
	dest->v->nu03 = src.nu03;
	dest->v->nu11 = src.nu11;
	dest->v->nu12 = src.nu12;
	dest->v->nu20 = src.nu20;
	dest->v->nu21 = src.nu21;
	dest->v->nu30 = src.nu30;

}


void Size_cpy(Size_t* dest, const Size src) {
	dest->v->height = src.height;
	dest->v->width = src.width;
}

void DMatch_cpy(DMatch_t* dest, const DMatch src) {
	dest->v->distance = src.distance;
	dest->v->imgIdx = src.imgIdx;
	dest->v->queryIdx = src.queryIdx;
	dest->v->trainIdx = src.trainIdx;
}

void CvDTreeNode_cpy(CvDTreeNode_t* dest, const CvDTreeNode* src) {
	dest->v->alpha = src->alpha;
	dest->v->buf_idx = src->buf_idx;
	dest->v->class_idx = src->class_idx;
	dest->v->complexity = src->complexity;
	dest->v->cv_node_error = src->cv_node_error;
	dest->v->cv_node_risk = src->cv_node_risk;
	dest->v->cv_Tn = src->cv_Tn;
	dest->v->depth = src->depth;
	dest->v->left = src->left;
	dest->v->maxlr = src->maxlr;
	dest->v->node_risk = src->node_risk;
	dest->v->num_valid = src->num_valid;
	dest->v->offset = src->offset;
	dest->v->parent = src->parent;
	dest->v->right = src->right;
	dest->v->sample_count = src->sample_count;
	dest->v->split = src->split;
	dest->v->Tn = src->Tn;
	dest->v->tree_error = src->tree_error;
	dest->v->tree_risk = src->tree_risk;
	dest->v->value = src->value;
}

void Point_cpy(Point_t* dest, const Point src) {
	dest->v->x = src.x;
	dest->v->y = src.y;
}

void Point2f_cpy(Point2f_t* dest, const Point2f src) {
	dest->v->x = src.x;
	dest->v->y = src.y;
}
void  Point2d_cpy(Point2d_t* dest, const  Point2d src) {
	dest->v->x = src.x;
	dest->v->y = src.y;
}
void  RotatedRect_cpy(RotatedRect_t* dest, const  RotatedRect src) {
	dest->v->angle = src.angle;
	dest->v->center.x = src.center.x;
	dest->v->center.y = src.center.y;
	dest->v->size.height = src.size.height;
	dest->v->size.width = src.size.width;
}

void Scalar_cpy(Scalar_t* dest, const Scalar src) {
	dest->v->val[0] = src.val[0];
	dest->v->val[1] = src.val[1];
	dest->v->val[2] = src.val[2];
	dest->v->val[3] = src.val[3];
}
void  Vec2d_cpy(Vec2d_t* dest, const  Vec2d src) {
	dest->v->val[0] = src.val[0];
	dest->v->val[1] = src.val[1];
}

void  Vec3d_cpy(Vec3d_t* dest, const  Vec3d src) {
	dest->v->val[0] = src.val[0];
	dest->v->val[1] = src.val[1];
	dest->v->val[2] = src.val[2];
}


void  Vec3b_cpy(Vec3b_t* dest, const  Vec3b src) {
	dest->v->val[0] = src.val[0];
	dest->v->val[1] = src.val[1];
	dest->v->val[2] = src.val[2];
}


void  Vec4f_cpy(Vec4f_t* dest, const  Vec4f src) {
	dest->v->val[0] = src.val[0];
	dest->v->val[1] = src.val[1];
	dest->v->val[2] = src.val[2];
	dest->v->val[3] = src.val[3];
}


void  Vec6f_cpy(Vec6f_t* dest, const  Vec6f src) {
	dest->v->val[0] = src.val[0];
	dest->v->val[1] = src.val[1];
	dest->v->val[2] = src.val[2];
	dest->v->val[3] = src.val[3];
	dest->v->val[4] = src.val[4];
	dest->v->val[5] = src.val[5];
}


void  Rect_cpy(Rect_t* dest, const  Rect src) {
	dest->v->height = src.height;
	dest->v->width = src.width;
	dest->v->x = src.x;
	dest->v->y = src.y;
}

void CvTermCriteria_cpy(CvTermCriteria_t* dest, const CvTermCriteria src) {
	dest->v->epsilon = src.epsilon;
	dest->v->max_iter = src.max_iter;
	dest->v->type = src.type;
}

void KeyPoint_cpy(KeyPoint_t* dest, const KeyPoint src) {
	dest->v->angle = src.angle;
	dest->v->class_id = src.class_id;
	dest->v->octave = src.octave;
	dest->v->pt.x = src.pt.x;
	dest->v->pt.y = src.pt.y;
	dest->v->response = src.response;
	dest->v->size = src.size;
}




//**************************************************************************************
// ---------------------- string class wrapper ---------------------------------

CVAPI(string_t*)   pCvStringCreate(const int nrchar)
{
	string_t* wrapper = new string_t;

try {
	wrapper->v = (char*)cvAlloc((nrchar + 1) * sizeof(char));
	wrapper->nrchar = nrchar;
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};
	return (wrapper);
}

CVAPI(void)   pCvStringDelete(string_t* wrapper)
{
	assert(wrapper);
	cvFree_((void*)wrapper->v);
	delete wrapper;
}


// ---------------------- Mat class wrapper ---------------------------------
void Mat_cpy(Mat_t* dest, Mat src) {
	src.copyTo(*dest->v);
}

CVAPI(struct  Mat_t*)   pCvMatCreate(int ndims, const int* dims, int mtype)
{
	struct Mat_t* wrapper = new Mat_t;
	wrapper->v = new cv::Mat(ndims, dims, mtype, 0.0f);
	return (wrapper);
}

CVAPI(struct  Mat_t*)   pCvMatCreateEmpty()
{
	struct Mat_t* wrapper = new Mat_t;
	wrapper->v = new cv::Mat();
	return (wrapper);
}

CVAPI(struct  Mat_t*)   pCvMat2dCreate(int cols, int rows, int mtype)
{
	struct Mat_t* wrapper = new Mat_t;
	wrapper->v = new cv::Mat(cols, rows, mtype, 0.0f);
	return (wrapper);
}

CVAPI(struct  Mat_t*)   pCvMatImageCreate(int width, int height, int mtype)
{
	struct Mat_t* wrapper = new Mat_t;
	wrapper->v = new cv::Mat(Size(width, height), mtype, Scalar(0.0f));
	return (wrapper);
}

CVAPI(struct  Mat_t*)   pCvMatROI(Mat_t* src, CvRectS* roi)
{
	struct Mat_t* wrapper = new Mat_t;
	wrapper->v = new cv::Mat();
	Mat m = (*src->v)(cv::Rect(roi->x, roi->y, roi->width, roi->height));
	Mat_cpy(wrapper, m);
	return (wrapper);
}

CVAPI(void)   pCvMatDelete(struct Mat_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}

CVAPI(void)   pCvMatFill(struct Mat_t* wrapper, struct Scalar_t* val)
{
	wrapper->v->setTo(*val->v);
}

CVAPI(void)   pCvMatCopy(struct Mat_t* src, struct Mat_t* dst)
{
	(src->v)->copyTo(*dst->v);
}


CVAPI(unsigned char)   pCvMatGetByte(struct Mat_t* wrapper, int rowind, int colind) {
	return wrapper->v->at<unsigned char>(rowind, colind);
}

CVAPI(int)   pCvMatGetInt(struct Mat_t* wrapper, int rowind, int colind) {
	return wrapper->v->at<int>(rowind, colind);
}

CVAPI(float)   pCvMatGetFloat(struct Mat_t* wrapper, int rowind, int colind) {
	return wrapper->v->at<float>(rowind, colind);
}

CVAPI(float)   pCvMatGetFloatMultidim(struct Mat_t* wrapper, int* indexes) {
	return wrapper->v->at<float>(indexes);
}

CVAPI(double)   pCvMatGetDouble(struct Mat_t* wrapper, int rowind, int colind) {
	return wrapper->v->at<double>(rowind, colind);
}


CVAPI(Vec3b_t*)   pCvMatGetPixelC3(struct Mat_t* wrapper, int rowind, int colind) {
	Vec3b_t* retval = new Vec3b_t;
	Vec3b* pr = new Vec3b();
	retval->v = pr;
	Vec3b p = wrapper->v->at<Vec3b>(rowind, colind);
	Vec3b_cpy(retval, p);
	return (retval);
}


CVAPI(unsigned char)   pCvMatSetByte(struct Mat_t* wrapper, int rowind, int colind, unsigned char value) {
	wrapper->v->at<unsigned char>(rowind, colind) = value;
	return (value);
}

CVAPI(int)   pCvMatSetInt(struct Mat_t* wrapper, int rowind, int colind, int value) {
	wrapper->v->at<int>(rowind, colind) = value;
	return (value);
}

CVAPI(float)   pCvMatSetFloat(struct Mat_t* wrapper, int rowind, int colind, float value) {
	wrapper->v->at<float>(rowind, colind) = value;
	return (value);
}

CVAPI(float)   pCvMatSetFloatMultidim(struct Mat_t* wrapper, int* indexes, float value) {
	wrapper->v->at<float>(indexes) = value;
	return (value);
}


CVAPI(double)   pCvMatSetDouble(struct Mat_t* wrapper, int rowind, int colind, double value) {
	wrapper->v->at<double>(rowind, colind) = value;
	return (value);
}


CVAPI(Vec3b_t*)   pCvMatSetPixelC3(struct Mat_t* wrapper, int rowind, int colind, Vec3b_t* value) {
	wrapper->v->at<Vec3b>(rowind, colind) = *value->v;
	return (value);
}

CVAPI(int)   pCvMatGetWidth(struct Mat_t* wrapper) {
	return wrapper->v->cols;
}

CVAPI(int)   pCvMatGetHeight(struct Mat_t* wrapper) {
	return wrapper->v->rows;
}

CVAPI(int)   pCvMatGetChannels(struct Mat_t* wrapper) {
	return wrapper->v->channels();
}

CVAPI(int)   pCvMatGetType(struct Mat_t* wrapper) {
	return wrapper->v->type();
}

CVAPI(int)   pCvMatGetDims(struct Mat_t* wrapper) {
	return wrapper->v->dims;
}

CVAPI(int)   pCvMatGetData(struct Mat_t* wrapper) {
	return int(wrapper->v->data);
}

CVAPI(int)   pCvMatGetDepth(struct Mat_t* wrapper) {
	return wrapper->v->depth();
}

CVAPI(struct Mat_t*)   pCvMatGetRow(struct Mat_t* wrapper, int nrow) {
	Mat tr = (*wrapper->v).row(nrow);
	Mat*  t = new Mat();
	Mat_t*  retval = new Mat_t;
	retval->v = t;
	Mat_cpy(retval, tr);

	return (retval);
}

CVAPI(IplImage*)  pCvMatToIplimage(Mat_t* srcmat)
{
	IplImage* pipl = new IplImage;
	// converte direttamente cv::Mat in nuova iplimage
	*pipl = *srcmat->v;
	return pipl;

}

CVAPI(void)   pCvIplImageDelete(IplImage* pipl)
{
	assert(pipl);
	delete pipl;
}

CVAPI(void) pCvIplImageToMat(IplImage* pipl, Mat_t* outmat) {
	// converte iplimage in nuova Mat, e copia i dati (solo OCV 2)
	//Mat newmat = Mat(&src, true);
	// od anche meglio, supportato in OCV 3+ ....
	// release previous Mat object
	*outmat->v = cv::cvarrToMat(pipl, true);
}
//**************************************************************************************


// ---------------------- Size class wrapper ---------------------------------
CVAPI(Size_t*)   pCvSizeCreate()
{
	Size_t* wrapper = new  Size_t;
	try {
		wrapper->v = new  Size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvSizeDelete(Size_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvSizeToStruct(Size_t* wrapper, CvSizeS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->width = wrapper->v->width;
		dest->height = wrapper->v->height;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvSizeFromStruct(Size_t* wrapper, CvSizeS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->width = src->width;
		wrapper->v->height = src->height;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Rect class wrapper ---------------------------------
CVAPI(Rect_t*)   pCvRectCreate()
{
	Rect_t* wrapper = new  Rect_t;
	try {
		wrapper->v = new  Rect();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvRectDelete(Rect_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvRectToStruct(Rect_t* wrapper, CvRectS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->x = wrapper->v->x;
		dest->y = wrapper->v->y;
		dest->width = wrapper->v->width;
		dest->height = wrapper->v->height;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvRectFromStruct(Rect_t* wrapper, CvRectS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->x = src->x;
		wrapper->v->y = src->y;
		wrapper->v->width = src->width;
		wrapper->v->height = src->height;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Range class wrapper ---------------------------------
CVAPI(Range_t*)   pCvRangeCreate()
{
	Range_t* wrapper = new  Range_t;
	try {
		wrapper->v = new  Range();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvRangeDelete(Range_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvRangeToStruct(Range_t* wrapper, CvRangeS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->start = wrapper->v->start;
		dest->end = wrapper->v->end;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvRangeFromStruct(Range_t* wrapper, CvRangeS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->start = src->start;
		wrapper->v->end = src->end;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- CvSlice class wrapper ---------------------------------
CVAPI(CvSlice_t*)   pCvCvSliceCreate()
{
	CvSlice_t* wrapper = new  CvSlice_t;
	try {
		wrapper->v = new  CvSlice();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvCvSliceDelete(CvSlice_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvCvSliceToStruct(CvSlice_t* wrapper, CvSliceS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->start = wrapper->v->start_index;
		dest->end = wrapper->v->end_index;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvCvSliceFromStruct(CvSlice_t* wrapper, CvSliceS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->start_index = src->start;
		wrapper->v->end_index = src->end;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Point class wrapper ---------------------------------
CVAPI(Point_t*)   pCvPointCreate()
{
	Point_t* wrapper = new  Point_t;
	try {
		wrapper->v = new  Point();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvPointDelete(Point_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvPointToStruct(Point_t* wrapper, CvPointS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->x = wrapper->v->x;
		dest->y = wrapper->v->y;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvPointFromStruct(Point_t* wrapper, CvPointS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->x = src->x;
		wrapper->v->y = src->y;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Point2f class wrapper ---------------------------------
CVAPI(Point2f_t*)   pCvPoint2fCreate()
{
	Point2f_t* wrapper = new  Point2f_t;
	try {
		wrapper->v = new  Point2f();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvPoint2fDelete(Point2f_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvPoint2fToStruct(Point2f_t* wrapper, CvPoint2fS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->x = wrapper->v->x;
		dest->y = wrapper->v->y;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvPoint2fFromStruct(Point2f_t* wrapper, CvPoint2fS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->x = src->x;
		wrapper->v->y = src->y;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Point2d class wrapper ---------------------------------
CVAPI(Point2d_t*)   pCvPoint2dCreate()
{
	Point2d_t* wrapper = new  Point2d_t;
	try {
		wrapper->v = new  Point2d();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvPoint2dDelete(Point2d_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvPoint2dToStruct(Point2d_t* wrapper, CvPoint2dS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->x = wrapper->v->x;
		dest->y = wrapper->v->y;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvPoint2dFromStruct(Point2d_t* wrapper, CvPoint2dS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->x = src->x;
		wrapper->v->y = src->y;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Vec2d class wrapper ---------------------------------
CVAPI(Vec2d_t*)   pCvVec2dCreate()
{
	Vec2d_t* wrapper = new  Vec2d_t;
	try {
		wrapper->v = new  Vec2d();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVec2dDelete(Vec2d_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvVec2dToStruct(Vec2d_t* wrapper, CvVec2dS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->v0 = wrapper->v->val[0];
		dest->v1 = wrapper->v->val[1];

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvVec2dFromStruct(Vec2d_t* wrapper, CvVec2dS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->val[0] = src->v0;
		wrapper->v->val[1] = src->v1;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Vec3d class wrapper ---------------------------------
CVAPI(Vec3d_t*)   pCvVec3dCreate()
{
	Vec3d_t* wrapper = new  Vec3d_t;
	try {
		wrapper->v = new  Vec3d();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVec3dDelete(Vec3d_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvVec3dToStruct(Vec3d_t* wrapper, CvVec3dS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->v0 = wrapper->v->val[0];
		dest->v1 = wrapper->v->val[1];
		dest->v2 = wrapper->v->val[2];

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvVec3dFromStruct(Vec3d_t* wrapper, CvVec3dS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->val[0] = src->v0;
		wrapper->v->val[1] = src->v1;
		wrapper->v->val[2] = src->v2;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Vec3b class wrapper ---------------------------------
CVAPI(Vec3b_t*)   pCvVec3bCreate()
{
	Vec3b_t* wrapper = new  Vec3b_t;
	try {
		wrapper->v = new  Vec3b();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVec3bDelete(Vec3b_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvVec3bToStruct(Vec3b_t* wrapper, CvVec3bS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->v0 = wrapper->v->val[0];
		dest->v1 = wrapper->v->val[1];
		dest->v2 = wrapper->v->val[2];

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvVec3bFromStruct(Vec3b_t* wrapper, CvVec3bS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->val[0] = src->v0;
		wrapper->v->val[1] = src->v1;
		wrapper->v->val[2] = src->v2;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Vec4f class wrapper ---------------------------------
CVAPI(Vec4f_t*)   pCvVec4fCreate()
{
	Vec4f_t* wrapper = new  Vec4f_t;
	try {
		wrapper->v = new  Vec4f();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVec4fDelete(Vec4f_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvVec4fToStruct(Vec4f_t* wrapper, CvVec4fS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->v0 = wrapper->v->val[0];
		dest->v1 = wrapper->v->val[1];
		dest->v2 = wrapper->v->val[2];
		dest->v3 = wrapper->v->val[3];

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvVec4fFromStruct(Vec4f_t* wrapper, CvVec4fS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->val[0] = src->v0;
		wrapper->v->val[1] = src->v1;
		wrapper->v->val[2] = src->v2;
		wrapper->v->val[3] = src->v3;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Vec6f class wrapper ---------------------------------
CVAPI(Vec6f_t*)   pCvVec6fCreate()
{
	Vec6f_t* wrapper = new  Vec6f_t;
	try {
		wrapper->v = new  Vec6f();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVec6fDelete(Vec6f_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvVec6fToStruct(Vec6f_t* wrapper, CvVec6fS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->v0 = wrapper->v->val[0];
		dest->v1 = wrapper->v->val[1];
		dest->v2 = wrapper->v->val[2];
		dest->v3 = wrapper->v->val[3];
		dest->v4 = wrapper->v->val[4];
		dest->v5 = wrapper->v->val[5];

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvVec6fFromStruct(Vec6f_t* wrapper, CvVec6fS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->val[0] = src->v0;
		wrapper->v->val[1] = src->v1;
		wrapper->v->val[2] = src->v2;
		wrapper->v->val[3] = src->v3;
		wrapper->v->val[4] = src->v4;
		wrapper->v->val[5] = src->v5;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Scalar class wrapper ---------------------------------
CVAPI(Scalar_t*)   pCvScalarCreate()
{
	Scalar_t* wrapper = new  Scalar_t;
	try {
		wrapper->v = new  Scalar();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvScalarDelete(Scalar_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvScalarToStruct(Scalar_t* wrapper, CvScalarS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->v0 = wrapper->v->val[0];
		dest->v1 = wrapper->v->val[1];
		dest->v2 = wrapper->v->val[2];
		dest->v3 = wrapper->v->val[3];

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvScalarFromStruct(Scalar_t* wrapper, CvScalarS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->val[0] = src->v0;
		wrapper->v->val[1] = src->v1;
		wrapper->v->val[2] = src->v2;
		wrapper->v->val[3] = src->v3;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- RotatedRect class wrapper ---------------------------------
CVAPI(RotatedRect_t*)   pCvRotatedRectCreate()
{
	RotatedRect_t* wrapper = new  RotatedRect_t;
	try {
		wrapper->v = new  RotatedRect();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvRotatedRectDelete(RotatedRect_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvRotatedRectToStruct(RotatedRect_t* wrapper, CvRotatedRectS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->center.x = wrapper->v->center.x;
		dest->center.y = wrapper->v->center.y;
		dest->size.width = wrapper->v->size.width;
		dest->size.height = wrapper->v->size.height;
		dest->angle = wrapper->v->angle;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvRotatedRectFromStruct(RotatedRect_t* wrapper, CvRotatedRectS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->center.x = src->center.x;
		wrapper->v->center.y = src->center.y;
		wrapper->v->size.width = src->size.width;
		wrapper->v->size.height = src->size.height;
		wrapper->v->angle = src->angle;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- CvDTreeNode class wrapper ---------------------------------
CVAPI(CvDTreeNode_t*)   pCvCvDTreeNodeCreate()
{
	CvDTreeNode_t* wrapper = new  CvDTreeNode_t;
	try {
		wrapper->v = new  CvDTreeNode();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvCvDTreeNodeDelete(CvDTreeNode_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvCvDTreeNodeToStruct(CvDTreeNode_t* wrapper, CvDTreeNodeS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->class_idx = wrapper->v->class_idx;
		dest->Tn = wrapper->v->Tn;
		dest->value = wrapper->v->value;
		dest->parent = wrapper->v->parent;
		dest->left = wrapper->v->left;
		dest->right = wrapper->v->right;
		dest->split = wrapper->v->split;
		dest->sample_count = wrapper->v->sample_count;
		dest->depth = wrapper->v->depth;
		dest->num_valid = wrapper->v->num_valid;
		dest->offset = wrapper->v->offset;
		dest->buf_idx = wrapper->v->buf_idx;
		dest->maxlr = wrapper->v->maxlr;
		dest->complexity = wrapper->v->complexity;
		dest->alpha = wrapper->v->alpha;
		dest->node_risk = wrapper->v->node_risk;
		dest->tree_risk = wrapper->v->tree_risk;
		dest->tree_error = wrapper->v->tree_error;
		dest->cv_Tn = wrapper->v->cv_Tn;
		dest->cv_node_risk = wrapper->v->cv_node_risk;
		dest->cv_node_error = wrapper->v->cv_node_error;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvCvDTreeNodeFromStruct(CvDTreeNode_t* wrapper, CvDTreeNodeS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->class_idx = src->class_idx;
		wrapper->v->Tn = src->Tn;
		wrapper->v->value = src->value;
		wrapper->v->parent = (CvDTreeNode*) src->parent;
		wrapper->v->left = (CvDTreeNode*) src->left;
		wrapper->v->right = (CvDTreeNode*) src->right;
		wrapper->v->split = (CvDTreeSplit*) src->split;
		wrapper->v->sample_count = src->sample_count;
		wrapper->v->depth = src->depth;
		wrapper->v->num_valid = src->num_valid;
		wrapper->v->offset = src->offset;
		wrapper->v->buf_idx = src->buf_idx;
		wrapper->v->maxlr = src->maxlr;
		wrapper->v->complexity = src->complexity;
		wrapper->v->alpha = src->alpha;
		wrapper->v->node_risk = src->node_risk;
		wrapper->v->tree_risk = src->tree_risk;
		wrapper->v->tree_error = src->tree_error;
		wrapper->v->cv_Tn = src->cv_Tn;
		wrapper->v->cv_node_risk = src->cv_node_risk;
		wrapper->v->cv_node_error = src->cv_node_error;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- TermCriteria class wrapper ---------------------------------
CVAPI(TermCriteria_t*)   pCvTermCriteriaCreate()
{
	TermCriteria_t* wrapper = new  TermCriteria_t;
	try {
		wrapper->v = new  TermCriteria();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvTermCriteriaDelete(TermCriteria_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvTermCriteriaToStruct(TermCriteria_t* wrapper, CvTermCriteriaS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->type = wrapper->v->type;
		dest->max_iter = wrapper->v->maxCount;
		dest->epsilon = wrapper->v->epsilon;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvTermCriteriaFromStruct(TermCriteria_t* wrapper, CvTermCriteriaS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->type = src->type;
		wrapper->v->maxCount = src->max_iter;
		wrapper->v->epsilon = src->epsilon;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- CvTermCriteria class wrapper ---------------------------------
CVAPI(CvTermCriteria_t*)   pCvCvTermCriteriaCreate()
{
	CvTermCriteria_t* wrapper = new  CvTermCriteria_t;
	try {
		wrapper->v = new  CvTermCriteria();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvCvTermCriteriaDelete(CvTermCriteria_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvCvTermCriteriaToStruct(CvTermCriteria_t* wrapper, CvTermCriteriaS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->type = wrapper->v->type;
		dest->max_iter = wrapper->v->max_iter;
		dest->epsilon = wrapper->v->epsilon;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvCvTermCriteriaFromStruct(CvTermCriteria_t* wrapper, CvTermCriteriaS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->type = src->type;
		wrapper->v->max_iter = src->max_iter;
		wrapper->v->epsilon = src->epsilon;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- Moments class wrapper ---------------------------------
CVAPI(Moments_t*)   pCvMomentsCreate()
{
	Moments_t* wrapper = new  Moments_t;
	try {
		wrapper->v = new  Moments();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvMomentsDelete(Moments_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}CVAPI(void)   pCvMomentsToStruct(Moments_t* wrapper, CvMomentsS* dest)
{
	try {
		assert(wrapper);
		assert(dest);
		dest->m00 = wrapper->v->m00;
		dest->m10 = wrapper->v->m10;
		dest->m01 = wrapper->v->m01;
		dest->m20 = wrapper->v->m20;
		dest->m11 = wrapper->v->m11;
		dest->m30 = wrapper->v->m30;
		dest->m21 = wrapper->v->m21;
		dest->m12 = wrapper->v->m12;
		dest->m03 = wrapper->v->m03;
		dest->mu20 = wrapper->v->mu20;
		dest->mu11 = wrapper->v->mu11;
		dest->mu02 = wrapper->v->mu02;
		dest->mu30 = wrapper->v->mu30;
		dest->mu21 = wrapper->v->mu21;
		dest->mu12 = wrapper->v->mu12;
		dest->mu03 = wrapper->v->mu03;
		dest->nu20 = wrapper->v->nu20;
		dest->nu11 = wrapper->v->nu11;
		dest->nu02 = wrapper->v->nu02;
		dest->nu30 = wrapper->v->nu30;
		dest->nu21 = wrapper->v->nu21;
		dest->nu12 = wrapper->v->nu12;
		dest->nu03 = wrapper->v->nu03;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}CVAPI(void)   pCvMomentsFromStruct(Moments_t* wrapper, CvMomentsS* src)
{
	try {
		assert(wrapper);
		assert(src);
		wrapper->v->m00 = src->m00;
		wrapper->v->m10 = src->m10;
		wrapper->v->m01 = src->m01;
		wrapper->v->m20 = src->m20;
		wrapper->v->m11 = src->m11;
		wrapper->v->m30 = src->m30;
		wrapper->v->m21 = src->m21;
		wrapper->v->m12 = src->m12;
		wrapper->v->m03 = src->m03;
		wrapper->v->mu20 = src->mu20;
		wrapper->v->mu11 = src->mu11;
		wrapper->v->mu02 = src->mu02;
		wrapper->v->mu30 = src->mu30;
		wrapper->v->mu21 = src->mu21;
		wrapper->v->mu12 = src->mu12;
		wrapper->v->mu03 = src->mu03;
		wrapper->v->nu20 = src->nu20;
		wrapper->v->nu11 = src->nu11;
		wrapper->v->nu02 = src->nu02;
		wrapper->v->nu30 = src->nu30;
		wrapper->v->nu21 = src->nu21;
		wrapper->v->nu12 = src->nu12;
		wrapper->v->nu03 = src->nu03;

	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
}

// ---------------------- IndexParams class wrapper ---------------------------------
CVAPI(IndexParams_t*)   pCvIndexParamsCreate()
{
	IndexParams_t* wrapper = new  IndexParams_t;
	try {
		wrapper->v = new  flann::IndexParams();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvIndexParamsDelete(IndexParams_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}
// ---------------------- SearchParams class wrapper ---------------------------------
CVAPI(SearchParams_t*)   pCvSearchParamsCreate()
{
	SearchParams_t* wrapper = new  SearchParams_t;
	try {
		wrapper->v = new  flann::SearchParams();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvSearchParamsDelete(SearchParams_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}
// ---------------------- CvSVMParams class wrapper ---------------------------------
CVAPI(CvSVMParams_t*)   pCvCvSVMParamsCreate()
{
	CvSVMParams_t* wrapper = new  CvSVMParams_t;
	try {
		wrapper->v = new  CvSVMParams();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvCvSVMParamsDelete(CvSVMParams_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}
// ---------------------- CvParamGrid class wrapper ---------------------------------
CVAPI(CvParamGrid_t*)   pCvCvParamGridCreate()
{
	CvParamGrid_t* wrapper = new  CvParamGrid_t;
	try {
		wrapper->v = new  CvParamGrid();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvCvParamGridDelete(CvParamGrid_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}
// ---------------------- CvGBTreesParams class wrapper ---------------------------------
CVAPI(CvGBTreesParams_t*)   pCvCvGBTreesParamsCreate()
{
	CvGBTreesParams_t* wrapper = new  CvGBTreesParams_t;
	try {
		wrapper->v = new  CvGBTreesParams();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvCvGBTreesParamsDelete(CvGBTreesParams_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}
// ---------------------- BackgroundSubtractor class wrapper ---------------------------------
CVAPI(BackgroundSubtractor_t*)   pCvBackgroundSubtractorCreate()
{
	BackgroundSubtractor_t* wrapper = new  BackgroundSubtractor_t;
	try {
		wrapper->v = new  BackgroundSubtractor();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvBackgroundSubtractorDelete(BackgroundSubtractor_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}
// ---------------------- CvStatModel class wrapper ---------------------------------
CVAPI(CvStatModel_t*)   pCvCvStatModelCreate()
{
	CvStatModel_t* wrapper = new  CvStatModel_t;
	try {
		wrapper->v = new  CvStatModel();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvCvStatModelDelete(CvStatModel_t* wrapper)
{
	assert(wrapper);
	delete wrapper->v;
	delete wrapper;
}
