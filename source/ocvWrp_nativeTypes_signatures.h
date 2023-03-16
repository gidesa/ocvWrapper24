#pragma once
struct  vector_uchar;
struct  vector_int;
struct  vector_float;
struct  vector_double;
struct  vector_Point;
struct  vector_Point2f;
struct  vector_Vec4f;
struct  vector_Vec6f;
struct  vector_Rect;
struct  vector_KeyPoint;
struct  vector_Mat;
struct  vector_DMatch;
struct  vector_string;
struct  vector_vector_Point;
struct  vector_vector_Point2f;
struct  vector_vector_DMatch;

struct  Ptr_Algorithm;
struct  Ptr_Feature2D; 
struct  Ptr_FaceRecognizer; 
struct  Ptr_CLAHE;  
struct  Ptr_FeatureDetector;
struct  Ptr_DescriptorExtractor;
struct  Ptr_DescriptorMatcher;
struct  Ptr_flann_IndexParams;
struct  Ptr_flann_SearchParams;

struct  SimpleBlobDetector_Params;

struct  cvflann_flann_distance_t;
struct  cvflann_flann_algorithm_t;

typedef char*  c_string;
typedef unsigned char uchar;
typedef long long int64;

struct Mat_t; 
struct string_t; 
struct Point2f_t; 
struct Point2d_t; 
struct Point_t; 
struct Size_t; 
struct Rect_t; 
struct Range_t; 
struct CvSlice_t; 
struct Vec2d_t; 
struct Vec3b_t; 
struct Vec3d_t; 
struct Vec4f_t; 
struct Vec6f_t; 
struct Scalar_t; 
struct RotatedRect_t; 
struct Filenode_t; 
struct TermCriteria_t; 
struct CvTermCriteria_t; 
struct CvDTreeNode_t; 
struct Moments_t; 
struct IndexParams_t; 
struct SearchParams_t; 


struct CvPoint2fS
{
	float x;
	float y;
};
struct CvPoint2dS
{
	double x;
	double y;
};
struct CvPointS
{
	int x;
	int y;
};
struct CvSizeS
{
	int width;
	int height;
};
struct CvRectS
{
	int x;
	int y;
	int width;
	int height;
};
struct CvRangeS
{
	int  start;
	int  end;
};
struct CvSliceS
{
	int  start;
	int  end;
};
struct CvVec2dS {
	double v0;
	double v1;
};
struct CvVec3dS {
	double v0;
	double v1;
	double v2;
};
struct CvVec3bS {
	uchar v0;
	uchar v1;
	uchar v2;
};
struct CvVec4fS {
	double v0;
	double v1;
	double v2;
	double v3;
};
struct CvVec6fS {
	double v0;
	double v1;
	double v2;
	double v3;
	double v4;
	double v5;
};
struct CvScalarS {
	double v0;
	double v1;
	double v2;
	double v3;
};
struct CvRotatedRectS {
	CvPoint2fS center; //< the rectangle mass center
	CvSizeS size; //< width and height of the rectangle
	float angle;    //< the rotation angle. When the angle is 0, 90, 180, 270 etc., the rectangle becomes an up-right rectangle.
};
struct CvFileNodeS
{
	int tag;
	void* info; /* type information
			(only for user-defined object, for others it is 0) */
	double f; /* scalar floating-point number */
	int i;    /* scalar integer number */
	int strlen;  /* text string */
	char* str;
	void* seq; /* sequence (ordered collection of file nodes) */
	void* map; /* map (collection of named file nodes) */
};

struct CvTermCriteriaS
{
	int    type;  /* may be combination of
					 CV_TERMCRIT_ITER
					 CV_TERMCRIT_EPS */
	int    max_iter;
	double epsilon;
};
struct CvDTreeNodeS
{
	int class_idx;
	int Tn;
	double value;

	void* parent;
	void* left;
	void* right;

	void* split;

	int sample_count;
	int depth;
	int* num_valid;
	int offset;
	int buf_idx;
	double maxlr;

	// global pruning data
	int complexity;
	double alpha;
	double node_risk, tree_risk, tree_error;

	// cross-validation pruning data
	int* cv_Tn;
	double* cv_node_risk;
	double* cv_node_error;
};
struct CvMomentsS
{
	double  m00, m10, m01, m20, m11, m30, m21, m12, m03; /* spatial moments */
	double  mu20, mu11, mu02, mu30, mu21, mu12, mu03; /* central moments */
	double  nu20, nu11, nu02, nu30, nu21, nu12, nu03;
};

