

void  vector_Mat_cpy(vector_Mat* dest, vector_Mat src) {
       *dest = src;
}

void  vector_float_cpy(vector_float* dest, vector_float src) {
       *dest = src;
}

void  vector_Point_cpy(vector_Point* dest, vector_Point src) {
	*dest = src;
}


void  vector_Point2f_cpy(vector_Point2f* dest, vector_Point2f src) {
	*dest = src;
}

void  vector_DMatch_cpy(vector_DMatch* dest, vector_DMatch src) {
	*dest = src;
}
//*****************************************************************************************************************




// ---------------------- vector_Mat wrapper ---------------------------------
CVAPI(vector_Mat*)   pCvVectorMatCreate(int n)
{
	vector_Mat* wrapper = 0;
	try {
		wrapper = new vector_Mat(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorMatDelete(vector_Mat* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorMatLength(vector_Mat* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(Mat_t*)   pCvVectorMatGet(vector_Mat* wrapper, int index)
{

	Mat_t* retval = 0;
	try {
		assert(wrapper);
		Mat tr = (*wrapper).at(index);
		retval = new Mat_t;
		Mat* t = new Mat();
		retval->v = t;
		Mat_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(Mat_t*)   pCvVectorMatSet(vector_Mat* wrapper, int index, Mat_t* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value->v;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_string wrapper ---------------------------------
CVAPI(vector_string*)   pCvVectorstringCreate(int n)
{
	vector_string* wrapper = 0;
	try {
		wrapper = new vector_string(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorstringDelete(vector_string* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorstringLength(vector_string* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(string_t*)   pCvVectorstringGet(vector_string* wrapper, int index)
{
	string_t* retval = 0;
	try {
		assert(wrapper);
		int l = (*wrapper).at(index).length();
		retval = pCvStringCreate(l);
		strcpy(retval->v, (*wrapper).at(index).c_str());
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(string_t*)   pCvVectorstringSet(vector_string* wrapper, int index, string_t* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value->v;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_KeyPoint wrapper ---------------------------------
CVAPI(vector_KeyPoint*)   pCvVectorKeyPointCreate(int n)
{
	vector_KeyPoint* wrapper = 0;
	try {
		wrapper = new vector_KeyPoint(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorKeyPointDelete(vector_KeyPoint* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorKeyPointLength(vector_KeyPoint* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(KeyPoint_t*)   pCvVectorKeyPointGet(vector_KeyPoint* wrapper, int index)
{

	KeyPoint_t* retval = 0;
	try {
		assert(wrapper);
		KeyPoint tr = (*wrapper).at(index);
		retval = new KeyPoint_t;
		KeyPoint* t = new KeyPoint();
		retval->v = t;
		KeyPoint_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(KeyPoint_t*)   pCvVectorKeyPointSet(vector_KeyPoint* wrapper, int index, KeyPoint_t* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value->v;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_float wrapper ---------------------------------
CVAPI(vector_float*)   pCvVectorfloatCreate(int n)
{
	vector_float* wrapper = 0;
	try {
		wrapper = new vector_float(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorfloatDelete(vector_float* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorfloatLength(vector_float* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(float)   pCvVectorfloatGet(vector_float* wrapper, int index)
{
	assert(wrapper);
	float retval = (*wrapper).at(index);
	return (retval);
}


CVAPI(float)   pCvVectorfloatSet(vector_float* wrapper, int index, float value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = value;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_int wrapper ---------------------------------
CVAPI(vector_int*)   pCvVectorintCreate(int n)
{
	vector_int* wrapper = 0;
	try {
		wrapper = new vector_int(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorintDelete(vector_int* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorintLength(vector_int* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(int)   pCvVectorintGet(vector_int* wrapper, int index)
{
	assert(wrapper);
	int retval = (*wrapper).at(index);
	return (retval);
}


CVAPI(int)   pCvVectorintSet(vector_int* wrapper, int index, int value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = value;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_uchar wrapper ---------------------------------
CVAPI(vector_uchar*)   pCvVectorucharCreate(int n)
{
	vector_uchar* wrapper = 0;
	try {
		wrapper = new vector_uchar(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorucharDelete(vector_uchar* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorucharLength(vector_uchar* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(uchar)   pCvVectorucharGet(vector_uchar* wrapper, int index)
{
	assert(wrapper);
	uchar retval = (*wrapper).at(index);
	return (retval);
}


CVAPI(uchar)   pCvVectorucharSet(vector_uchar* wrapper, int index, uchar value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = value;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_Rect wrapper ---------------------------------
CVAPI(vector_Rect*)   pCvVectorRectCreate(int n)
{
	vector_Rect* wrapper = 0;
	try {
		wrapper = new vector_Rect(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorRectDelete(vector_Rect* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorRectLength(vector_Rect* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(Rect_t*)   pCvVectorRectGet(vector_Rect* wrapper, int index)
{

	Rect_t* retval = 0;
	try {
		assert(wrapper);
		Rect tr = (*wrapper).at(index);
		retval = new Rect_t;
		Rect* t = new Rect();
		retval->v = t;
		Rect_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(Rect_t*)   pCvVectorRectSet(vector_Rect* wrapper, int index, Rect_t* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value->v;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_double wrapper ---------------------------------
CVAPI(vector_double*)   pCvVectordoubleCreate(int n)
{
	vector_double* wrapper = 0;
	try {
		wrapper = new vector_double(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectordoubleDelete(vector_double* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectordoubleLength(vector_double* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(double)   pCvVectordoubleGet(vector_double* wrapper, int index)
{
	assert(wrapper);
	double retval = (*wrapper).at(index);
	return (retval);
}


CVAPI(double)   pCvVectordoubleSet(vector_double* wrapper, int index, double value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = value;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_DMatch wrapper ---------------------------------
CVAPI(vector_DMatch*)   pCvVectorDMatchCreate(int n)
{
	vector_DMatch* wrapper = 0;
	try {
		wrapper = new vector_DMatch(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorDMatchDelete(vector_DMatch* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorDMatchLength(vector_DMatch* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(DMatch_t*)   pCvVectorDMatchGet(vector_DMatch* wrapper, int index)
{

	DMatch_t* retval = 0;
	try {
		assert(wrapper);
		DMatch tr = (*wrapper).at(index);
		retval = new DMatch_t;
		DMatch* t = new DMatch();
		retval->v = t;
		DMatch_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(DMatch_t*)   pCvVectorDMatchSet(vector_DMatch* wrapper, int index, DMatch_t* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value->v;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_Point wrapper ---------------------------------
CVAPI(vector_Point*)   pCvVectorPointCreate(int n)
{
	vector_Point* wrapper = 0;
	try {
		wrapper = new vector_Point(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorPointDelete(vector_Point* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorPointLength(vector_Point* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(Point_t*)   pCvVectorPointGet(vector_Point* wrapper, int index)
{

	Point_t* retval = 0;
	try {
		assert(wrapper);
		Point tr = (*wrapper).at(index);
		retval = new Point_t;
		Point* t = new Point();
		retval->v = t;
		Point_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(Point_t*)   pCvVectorPointSet(vector_Point* wrapper, int index, Point_t* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value->v;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_Point2f wrapper ---------------------------------
CVAPI(vector_Point2f*)   pCvVectorPoint2fCreate(int n)
{
	vector_Point2f* wrapper = 0;
	try {
		wrapper = new vector_Point2f(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorPoint2fDelete(vector_Point2f* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorPoint2fLength(vector_Point2f* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(Point2f_t*)   pCvVectorPoint2fGet(vector_Point2f* wrapper, int index)
{

	Point2f_t* retval = 0;
	try {
		assert(wrapper);
		Point2f tr = (*wrapper).at(index);
		retval = new Point2f_t;
		Point2f* t = new Point2f();
		retval->v = t;
		Point2f_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(Point2f_t*)   pCvVectorPoint2fSet(vector_Point2f* wrapper, int index, Point2f_t* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value->v;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_Vec4f wrapper ---------------------------------
CVAPI(vector_Vec4f*)   pCvVectorVec4fCreate(int n)
{
	vector_Vec4f* wrapper = 0;
	try {
		wrapper = new vector_Vec4f(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorVec4fDelete(vector_Vec4f* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorVec4fLength(vector_Vec4f* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(Vec4f_t*)   pCvVectorVec4fGet(vector_Vec4f* wrapper, int index)
{

	Vec4f_t* retval = 0;
	try {
		assert(wrapper);
		Vec4f tr = (*wrapper).at(index);
		retval = new Vec4f_t;
		Vec4f* t = new Vec4f();
		retval->v = t;
		Vec4f_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(Vec4f_t*)   pCvVectorVec4fSet(vector_Vec4f* wrapper, int index, Vec4f_t* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value->v;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_Vec6f wrapper ---------------------------------
CVAPI(vector_Vec6f*)   pCvVectorVec6fCreate(int n)
{
	vector_Vec6f* wrapper = 0;
	try {
		wrapper = new vector_Vec6f(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorVec6fDelete(vector_Vec6f* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorVec6fLength(vector_Vec6f* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(Vec6f_t*)   pCvVectorVec6fGet(vector_Vec6f* wrapper, int index)
{

	Vec6f_t* retval = 0;
	try {
		assert(wrapper);
		Vec6f tr = (*wrapper).at(index);
		retval = new Vec6f_t;
		Vec6f* t = new Vec6f();
		retval->v = t;
		Vec6f_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(Vec6f_t*)   pCvVectorVec6fSet(vector_Vec6f* wrapper, int index, Vec6f_t* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value->v;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_vector_Point wrapper ---------------------------------
CVAPI(vector_vector_Point*)   pCvVectorvector_PointCreate(int n)
{
	vector_vector_Point* wrapper = 0;
	try {
		wrapper = new vector_vector_Point(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorvector_PointDelete(vector_vector_Point* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorvector_PointLength(vector_vector_Point* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(vector_Point*)   pCvVectorvector_PointGet(vector_vector_Point* wrapper, int index)
{
	vector_Point* retval = 0;
	try {
		assert(wrapper);
		vector_Point tr = (*wrapper).at(index);
		retval = new vector_Point();
		vector_Point_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(vector_Point*)   pCvVectorvector_PointSet(vector_vector_Point* wrapper, int index, vector_Point* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_vector_DMatch wrapper ---------------------------------
CVAPI(vector_vector_DMatch*)   pCvVectorvector_DMatchCreate(int n)
{
	vector_vector_DMatch* wrapper = 0;
	try {
		wrapper = new vector_vector_DMatch(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorvector_DMatchDelete(vector_vector_DMatch* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorvector_DMatchLength(vector_vector_DMatch* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(vector_DMatch*)   pCvVectorvector_DMatchGet(vector_vector_DMatch* wrapper, int index)
{
	vector_DMatch* retval = 0;
	try {
		assert(wrapper);
		vector_DMatch tr = (*wrapper).at(index);
		retval = new vector_DMatch();
		vector_DMatch_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(vector_DMatch*)   pCvVectorvector_DMatchSet(vector_vector_DMatch* wrapper, int index, vector_DMatch* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}

// ---------------------- vector_vector_Point2f wrapper ---------------------------------
CVAPI(vector_vector_Point2f*)   pCvVectorvector_Point2fCreate(int n)
{
	vector_vector_Point2f* wrapper = 0;
	try {
		wrapper = new vector_vector_Point2f(n);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (wrapper);
}

CVAPI(void)   pCvVectorvector_Point2fDelete(vector_vector_Point2f* wrapper)
{
	assert(wrapper);
	delete wrapper;
}

CVAPI(int)   pCvVectorvector_Point2fLength(vector_vector_Point2f* wrapper)
{
	int retval = 0;
	try {
		retval = (*wrapper).size();
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(vector_Point2f*)   pCvVectorvector_Point2fGet(vector_vector_Point2f* wrapper, int index)
{
	vector_Point2f* retval = 0;
	try {
		assert(wrapper);
		vector_Point2f tr = (*wrapper).at(index);
		retval = new vector_Point2f();
		vector_Point2f_cpy(retval, tr);
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}


CVAPI(vector_Point2f*)   pCvVectorvector_Point2fSet(vector_vector_Point2f* wrapper, int index, vector_Point2f* value)
{
	try {
		assert(wrapper);
		(*wrapper).at(index) = *value;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return value;
}
