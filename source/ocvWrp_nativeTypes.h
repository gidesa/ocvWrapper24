#pragma once
typedef vector<uchar> vector_uchar;
typedef vector<int> vector_int;
typedef vector<float> vector_float;
typedef vector<double> vector_double;
typedef vector<Point> vector_Point;
typedef vector<Point2f> vector_Point2f;
typedef vector<Vec4f> vector_Vec4f;
typedef vector<Vec6f> vector_Vec6f;
typedef vector<Rect> vector_Rect;
typedef vector<KeyPoint> vector_KeyPoint;
typedef vector<Mat> vector_Mat;
typedef vector<DMatch> vector_DMatch;
typedef vector<string> vector_string;
typedef vector<vector<Point> > vector_vector_Point;
typedef vector<vector<Point2f> > vector_vector_Point2f;
typedef vector<vector<DMatch> > vector_vector_DMatch;

typedef Ptr<Algorithm> Ptr_Algorithm;
typedef Ptr<Feature2D> Ptr_Feature2D; 
typedef Ptr<FaceRecognizer> Ptr_FaceRecognizer; 
typedef Ptr<CLAHE> Ptr_CLAHE;  
typedef Ptr<FeatureDetector> Ptr_FeatureDetector;
typedef Ptr<DescriptorExtractor> Ptr_DescriptorExtractor;
typedef Ptr<DescriptorMatcher> Ptr_DescriptorMatcher;
typedef Ptr<flann::IndexParams> Ptr_flann_IndexParams;
typedef Ptr<flann::SearchParams> Ptr_flann_SearchParams;

typedef SimpleBlobDetector::Params SimpleBlobDetector_Params;

typedef cvflann::flann_distance_t cvflann_flann_distance_t;
typedef cvflann::flann_algorithm_t cvflann_flann_algorithm_t;

typedef char*  c_string;

struct Mat_t { cv::Mat*  v; };
struct string_t { char*  v; int nrchar; };
struct Point2f_t { Point2f*  v;  };
struct Point2d_t { Point2d*  v; };
struct Point_t { Point*  v; };
struct Size_t { Size*  v; };
struct Rect_t { Rect*  v; };
struct Range_t { Range*  v; };
struct CvSlice_t { CvSlice*  v; };
struct Vec2d_t { Vec2d*  v; };
struct Vec3b_t { Vec3b*  v; };
struct Vec3d_t { Vec3d*  v; };
struct Vec4f_t { Vec4f*  v; };
struct Vec6f_t { Vec6f*  v; };struct Scalar_t { Scalar*  v; };
struct RotatedRect_t { RotatedRect*  v; };
struct Filenode_t { FileNode* v; };
struct TermCriteria_t { TermCriteria* v; };
struct CvTermCriteria_t { CvTermCriteria* v; };
struct CvDTreeNode_t { CvDTreeNode* v; };
struct Moments_t { Moments* v; };
struct IndexParams_t { flann::IndexParams* v;  };
struct SearchParams_t { flann::SearchParams* v; };




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
    double  m00, m10, m01, m20, m11,  m30, m21, m12, m03; /* spatial moments */
    double  mu20, mu11, mu02, mu30, mu21, mu12, mu03; /* central moments */
    double  nu20, nu11, nu02, nu30, nu21, nu12, nu03;
};















