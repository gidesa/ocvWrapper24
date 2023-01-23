


// ---------------------- Ptr_Algorithm  wrapper ---------------------------------
CVAPI(Algorithm_t*)   pCvPtr_AlgorithmConvert(Ptr_Algorithm* wrapper)
{
	Algorithm_t* retval = 0;
	try {
		Ptr_Algorithm p = *wrapper;
		Algorithm*  c = p.obj;
		retval = new  Algorithm_t;
		retval->v = c;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(void)   pCvPtr_AlgorithmDelete(Ptr_Algorithm* ptr, Algorithm_t* wrapper)
{
	assert(ptr);
	if (wrapper != 0) { delete wrapper; };
	delete ptr;
}
// ---------------------- Ptr_Feature2D  wrapper ---------------------------------
CVAPI(Feature2D_t*)   pCvPtr_Feature2DConvert(Ptr_Feature2D* wrapper)
{
	Feature2D_t* retval = 0;
	try {
		Ptr_Feature2D p = *wrapper;
		Feature2D*  c = p.obj;
		retval = new  Feature2D_t;
		retval->v = c;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(void)   pCvPtr_Feature2DDelete(Ptr_Feature2D* ptr, Feature2D_t* wrapper)
{
	assert(ptr);
	if (wrapper != 0) { delete wrapper; };
	delete ptr;
}
// ---------------------- Ptr_FaceRecognizer  wrapper ---------------------------------
CVAPI(FaceRecognizer_t*)   pCvPtr_FaceRecognizerConvert(Ptr_FaceRecognizer* wrapper)
{
	FaceRecognizer_t* retval = 0;
	try {
		Ptr_FaceRecognizer p = *wrapper;
		FaceRecognizer*  c = p.obj;
		retval = new  FaceRecognizer_t;
		retval->v = c;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(void)   pCvPtr_FaceRecognizerDelete(Ptr_FaceRecognizer* ptr, FaceRecognizer_t* wrapper)
{
	assert(ptr);
	if (wrapper != 0) { delete wrapper; };
	delete ptr;
}
// ---------------------- Ptr_CLAHE  wrapper ---------------------------------
CVAPI(CLAHE_t*)   pCvPtr_CLAHEConvert(Ptr_CLAHE* wrapper)
{
	CLAHE_t* retval = 0;
	try {
		Ptr_CLAHE p = *wrapper;
		CLAHE*  c = p.obj;
		retval = new  CLAHE_t;
		retval->v = c;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(void)   pCvPtr_CLAHEDelete(Ptr_CLAHE* ptr, CLAHE_t* wrapper)
{
	assert(ptr);
	if (wrapper != 0) { delete wrapper; };
	delete ptr;
}
// ---------------------- Ptr_FeatureDetector  wrapper ---------------------------------
CVAPI(FeatureDetector_t*)   pCvPtr_FeatureDetectorConvert(Ptr_FeatureDetector* wrapper)
{
	FeatureDetector_t* retval = 0;
	try {
		Ptr_FeatureDetector p = *wrapper;
		FeatureDetector*  c = p.obj;
		retval = new  FeatureDetector_t;
		retval->v = c;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(void)   pCvPtr_FeatureDetectorDelete(Ptr_FeatureDetector* ptr, FeatureDetector_t* wrapper)
{
	assert(ptr);
	if (wrapper != 0) { delete wrapper; };
	delete ptr;
}
// ---------------------- Ptr_DescriptorExtractor  wrapper ---------------------------------
CVAPI(DescriptorExtractor_t*)   pCvPtr_DescriptorExtractorConvert(Ptr_DescriptorExtractor* wrapper)
{
	DescriptorExtractor_t* retval = 0;
	try {
		Ptr_DescriptorExtractor p = *wrapper;
		DescriptorExtractor*  c = p.obj;
		retval = new  DescriptorExtractor_t;
		retval->v = c;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(void)   pCvPtr_DescriptorExtractorDelete(Ptr_DescriptorExtractor* ptr, DescriptorExtractor_t* wrapper)
{
	assert(ptr);
	if (wrapper != 0) { delete wrapper; };
	delete ptr;
}
// ---------------------- Ptr_DescriptorMatcher  wrapper ---------------------------------
CVAPI(DescriptorMatcher_t*)   pCvPtr_DescriptorMatcherConvert(Ptr_DescriptorMatcher* wrapper)
{
	DescriptorMatcher_t* retval = 0;
	try {
		Ptr_DescriptorMatcher p = *wrapper;
		DescriptorMatcher*  c = p.obj;
		retval = new  DescriptorMatcher_t;
		retval->v = c;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(void)   pCvPtr_DescriptorMatcherDelete(Ptr_DescriptorMatcher* ptr, DescriptorMatcher_t* wrapper)
{
	assert(ptr);
	if (wrapper != 0) { delete wrapper; };
	delete ptr;
}
// ---------------------- Ptr_flann_IndexParams  wrapper ---------------------------------
CVAPI(IndexParams_t*)   pCvPtr_flann_IndexParamsConvert(Ptr_flann_IndexParams* wrapper)
{
	IndexParams_t* retval = 0;
	try {
		Ptr_flann_IndexParams p = *wrapper;
		flann::IndexParams*  c = p.obj;
		retval = new  IndexParams_t;
		retval->v = c;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(void)   pCvPtr_flann_IndexParamsDelete(Ptr_flann_IndexParams* ptr, IndexParams_t* wrapper)
{
	assert(ptr);
	if (wrapper != 0) { delete wrapper; };
	delete ptr;
}
// ---------------------- Ptr_flann_SearchParams  wrapper ---------------------------------
CVAPI(SearchParams_t*)   pCvPtr_flann_SearchParamsConvert(Ptr_flann_SearchParams* wrapper)
{
	SearchParams_t* retval = 0;
	try {
		Ptr_flann_SearchParams p = *wrapper;
		flann::SearchParams*  c = p.obj;
		retval = new  SearchParams_t;
		retval->v = c;
	}
	catch (std::exception &e) {
		exceptionDisplay(e.what());
	};
	return (retval);
}

CVAPI(void)   pCvPtr_flann_SearchParamsDelete(Ptr_flann_SearchParams* ptr, SearchParams_t* wrapper)
{
	assert(ptr);
	if (wrapper != 0) { delete wrapper; };
	delete ptr;
}
