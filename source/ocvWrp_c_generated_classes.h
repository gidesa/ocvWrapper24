CVAPI(struct  BFMatcher_t* )   pCvBFMatcherCreate(int normType, bool crossCheck)
{
 
    
    struct BFMatcher_t* wrapper  = new BFMatcher_t;
try {
    wrapper->v = new cv::BFMatcher(normType, crossCheck);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvBFMatcherDelete(struct BFMatcher_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  BOWImgDescriptorExtractor_t* )   pCvBOWImgDescriptorExtractorCreate(Ptr_DescriptorExtractor* dextractor, Ptr_DescriptorMatcher* dmatcher)
{
 
    
    struct BOWImgDescriptorExtractor_t* wrapper  = new BOWImgDescriptorExtractor_t;
try {
    wrapper->v = new cv::BOWImgDescriptorExtractor(*dextractor, *dmatcher);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvBOWImgDescriptorExtractorDelete(struct BOWImgDescriptorExtractor_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  BOWKMeansTrainer_t* )   pCvBOWKMeansTrainerCreate(int clusterCount, TermCriteria_t* termcrit, int attempts, int flags)
{
 
    
    struct BOWKMeansTrainer_t* wrapper  = new BOWKMeansTrainer_t;
try {
    wrapper->v = new cv::BOWKMeansTrainer(clusterCount, *termcrit->v, attempts, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvBOWKMeansTrainerDelete(struct BOWKMeansTrainer_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  BRISK_t* )   pCvBRISKCreate(int thresh, int octaves, float patternScale)
{
 
    
    struct BRISK_t* wrapper  = new BRISK_t;
try {
    wrapper->v = new cv::BRISK(thresh, octaves, patternScale);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  BRISK_t* )   pCvBRISKCreate2(vector_float* radiusList, vector_int* numberList, float dMax, float dMin, vector_int* indexChange)
{
 
    
    struct BRISK_t* wrapper  = new BRISK_t;
try {
    wrapper->v = new cv::BRISK(*radiusList, *numberList, dMax, dMin, *indexChange);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvBRISKDelete(struct BRISK_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  BackgroundSubtractorMOG_t* )   pCvBackgroundSubtractorMOGCreate()
{
 
    
    struct BackgroundSubtractorMOG_t* wrapper  = new BackgroundSubtractorMOG_t;
try {
    wrapper->v = new cv::BackgroundSubtractorMOG();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  BackgroundSubtractorMOG_t* )   pCvBackgroundSubtractorMOGCreate2(int history, int nmixtures, double backgroundRatio, double noiseSigma)
{
 
    
    struct BackgroundSubtractorMOG_t* wrapper  = new BackgroundSubtractorMOG_t;
try {
    wrapper->v = new cv::BackgroundSubtractorMOG(history, nmixtures, backgroundRatio, noiseSigma);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvBackgroundSubtractorMOGDelete(struct BackgroundSubtractorMOG_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  BackgroundSubtractorMOG2_t* )   pCvBackgroundSubtractorMOG2Create()
{
 
    
    struct BackgroundSubtractorMOG2_t* wrapper  = new BackgroundSubtractorMOG2_t;
try {
    wrapper->v = new cv::BackgroundSubtractorMOG2();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  BackgroundSubtractorMOG2_t* )   pCvBackgroundSubtractorMOG2Create2(int history, float varThreshold, bool bShadowDetection)
{
 
    
    struct BackgroundSubtractorMOG2_t* wrapper  = new BackgroundSubtractorMOG2_t;
try {
    wrapper->v = new cv::BackgroundSubtractorMOG2(history, varThreshold, bShadowDetection);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvBackgroundSubtractorMOG2Delete(struct BackgroundSubtractorMOG2_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  CascadeClassifier_t* )   pCvCascadeClassifierCreate()
{
 
    
    struct CascadeClassifier_t* wrapper  = new CascadeClassifier_t;
try {
    wrapper->v = new cv::CascadeClassifier();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  CascadeClassifier_t* )   pCvCascadeClassifierCreate2(string_t* filename)
{
 
    
    struct CascadeClassifier_t* wrapper  = new CascadeClassifier_t;
try {
    wrapper->v = new cv::CascadeClassifier(filename->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvCascadeClassifierDelete(struct CascadeClassifier_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  CvANN_MLP_t* )   pCvCvANN_MLPCreate()
{
 
    
    struct CvANN_MLP_t* wrapper  = new CvANN_MLP_t;
try {
    wrapper->v = new CvANN_MLP();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  CvANN_MLP_t* )   pCvCvANN_MLPCreate2(Mat_t* layerSizes, int activateFunc, double fparam1, double fparam2)
{
 
    
    struct CvANN_MLP_t* wrapper  = new CvANN_MLP_t;
try {
    wrapper->v = new CvANN_MLP(*layerSizes->v, activateFunc, fparam1, fparam2);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvCvANN_MLPDelete(struct CvANN_MLP_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  CvBoost_t* )   pCvCvBoostCreate()
{
 
    
    struct CvBoost_t* wrapper  = new CvBoost_t;
try {
    wrapper->v = new CvBoost();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  CvBoost_t* )   pCvCvBoostCreate2(Mat_t* trainData, int tflag, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, Mat_t* varType, Mat_t* missingDataMask, CvBoostParams_t* params)
{
 
    
    struct CvBoost_t* wrapper  = new CvBoost_t;
try {
    wrapper->v = new CvBoost(*trainData->v, tflag, *responses->v, *varIdx->v, *sampleIdx->v, *varType->v, *missingDataMask->v, *params->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvCvBoostDelete(struct CvBoost_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  CvDTree_t* )   pCvCvDTreeCreate()
{
 
    
    struct CvDTree_t* wrapper  = new CvDTree_t;
try {
    wrapper->v = new CvDTree();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvCvDTreeDelete(struct CvDTree_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  CvERTrees_t* )   pCvCvERTreesCreate()
{
 
    
    struct CvERTrees_t* wrapper  = new CvERTrees_t;
try {
    wrapper->v = new CvERTrees();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvCvERTreesDelete(struct CvERTrees_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  CvGBTrees_t* )   pCvCvGBTreesCreate()
{
 
    
    struct CvGBTrees_t* wrapper  = new CvGBTrees_t;
try {
    wrapper->v = new CvGBTrees();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  CvGBTrees_t* )   pCvCvGBTreesCreate2(Mat_t* trainData, int tflag, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, Mat_t* varType, Mat_t* missingDataMask, CvGBTreesParams_t* params)
{
 
    
    struct CvGBTrees_t* wrapper  = new CvGBTrees_t;
try {
    wrapper->v = new CvGBTrees(*trainData->v, tflag, *responses->v, *varIdx->v, *sampleIdx->v, *varType->v, *missingDataMask->v, *params->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvCvGBTreesDelete(struct CvGBTrees_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  CvKNearest_t* )   pCvCvKNearestCreate()
{
 
    
    struct CvKNearest_t* wrapper  = new CvKNearest_t;
try {
    wrapper->v = new CvKNearest();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  CvKNearest_t* )   pCvCvKNearestCreate2(Mat_t* trainData, Mat_t* responses, Mat_t* sampleIdx, bool isRegression, int max_k)
{
 
    
    struct CvKNearest_t* wrapper  = new CvKNearest_t;
try {
    wrapper->v = new CvKNearest(*trainData->v, *responses->v, *sampleIdx->v, isRegression, max_k);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvCvKNearestDelete(struct CvKNearest_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  CvNormalBayesClassifier_t* )   pCvCvNormalBayesClassifierCreate()
{
 
    
    struct CvNormalBayesClassifier_t* wrapper  = new CvNormalBayesClassifier_t;
try {
    wrapper->v = new CvNormalBayesClassifier();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  CvNormalBayesClassifier_t* )   pCvCvNormalBayesClassifierCreate2(Mat_t* trainData, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx)
{
 
    
    struct CvNormalBayesClassifier_t* wrapper  = new CvNormalBayesClassifier_t;
try {
    wrapper->v = new CvNormalBayesClassifier(*trainData->v, *responses->v, *varIdx->v, *sampleIdx->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvCvNormalBayesClassifierDelete(struct CvNormalBayesClassifier_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  CvRTrees_t* )   pCvCvRTreesCreate()
{
 
    
    struct CvRTrees_t* wrapper  = new CvRTrees_t;
try {
    wrapper->v = new CvRTrees();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvCvRTreesDelete(struct CvRTrees_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  CvSVM_t* )   pCvCvSVMCreate()
{
 
    
    struct CvSVM_t* wrapper  = new CvSVM_t;
try {
    wrapper->v = new CvSVM();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  CvSVM_t* )   pCvCvSVMCreate2(Mat_t* trainData, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, CvSVMParams_t* params)
{
 
    
    struct CvSVM_t* wrapper  = new CvSVM_t;
try {
    wrapper->v = new CvSVM(*trainData->v, *responses->v, *varIdx->v, *sampleIdx->v, *params->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvCvSVMDelete(struct CvSVM_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  DMatch_t* )   pCvDMatchCreate()
{
 
    
    struct DMatch_t* wrapper  = new DMatch_t;
try {
    wrapper->v = new cv::DMatch();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  DMatch_t* )   pCvDMatchCreate2(int _queryIdx, int _trainIdx, float _distance)
{
 
    
    struct DMatch_t* wrapper  = new DMatch_t;
try {
    wrapper->v = new cv::DMatch(_queryIdx, _trainIdx, _distance);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  DMatch_t* )   pCvDMatchCreate3(int _queryIdx, int _trainIdx, int _imgIdx, float _distance)
{
 
    
    struct DMatch_t* wrapper  = new DMatch_t;
try {
    wrapper->v = new cv::DMatch(_queryIdx, _trainIdx, _imgIdx, _distance);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvDMatchDelete(struct DMatch_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  EM_t* )   pCvEMCreate(int nclusters, int covMatType, TermCriteria_t* termCrit)
{
 
    
    struct EM_t* wrapper  = new EM_t;
try {
    wrapper->v = new cv::EM(nclusters, covMatType, *termCrit->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvEMDelete(struct EM_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  FastFeatureDetector_t* )   pCvFastFeatureDetectorCreate(int threshold, bool nonmaxSuppression)
{
 
    
    struct FastFeatureDetector_t* wrapper  = new FastFeatureDetector_t;
try {
    wrapper->v = new cv::FastFeatureDetector(threshold, nonmaxSuppression);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvFastFeatureDetectorDelete(struct FastFeatureDetector_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  FileNode_t* )   pCvFileNodeCreate()
{
 
    
    struct FileNode_t* wrapper  = new FileNode_t;
try {
    wrapper->v = new cv::FileNode();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvFileNodeDelete(struct FileNode_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  FileStorage_t* )   pCvFileStorageCreate()
{
 
    
    struct FileStorage_t* wrapper  = new FileStorage_t;
try {
    wrapper->v = new cv::FileStorage();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  FileStorage_t* )   pCvFileStorageCreate2(string_t* source, int flags, string_t* encoding)
{
 
    
    struct FileStorage_t* wrapper  = new FileStorage_t;
try {
    wrapper->v = new cv::FileStorage(source->v, flags, encoding->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvFileStorageDelete(struct FileStorage_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  FlannBasedMatcher_t* )   pCvFlannBasedMatcherCreate(Ptr_flann_IndexParams* indexParams, Ptr_flann_SearchParams* searchParams)
{
 
    
    struct FlannBasedMatcher_t* wrapper  = new FlannBasedMatcher_t;
try {
    wrapper->v = new cv::FlannBasedMatcher(*indexParams, *searchParams);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvFlannBasedMatcherDelete(struct FlannBasedMatcher_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  GFTTDetector_t* )   pCvGFTTDetectorCreate(int maxCorners, double qualityLevel, double minDistance, int blockSize, bool useHarrisDetector, double k)
{
 
    
    struct GFTTDetector_t* wrapper  = new GFTTDetector_t;
try {
    wrapper->v = new cv::GFTTDetector(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector, k);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvGFTTDetectorDelete(struct GFTTDetector_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  GridAdaptedFeatureDetector_t* )   pCvGridAdaptedFeatureDetectorCreate(Ptr_FeatureDetector* detector, int maxTotalKeypoints, int gridRows, int gridCols)
{
 
    
    struct GridAdaptedFeatureDetector_t* wrapper  = new GridAdaptedFeatureDetector_t;
try {
    wrapper->v = new cv::GridAdaptedFeatureDetector(*detector, maxTotalKeypoints, gridRows, gridCols);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvGridAdaptedFeatureDetectorDelete(struct GridAdaptedFeatureDetector_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  HOGDescriptor_t* )   pCvHOGDescriptorCreate()
{
 
    
    struct HOGDescriptor_t* wrapper  = new HOGDescriptor_t;
try {
    wrapper->v = new cv::HOGDescriptor();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  HOGDescriptor_t* )   pCvHOGDescriptorCreate2(Size_t* _winSize, Size_t* _blockSize, Size_t* _blockStride, Size_t* _cellSize, int _nbins, int _derivAperture, double _winSigma, int _histogramNormType, double _L2HysThreshold, bool _gammaCorrection, int _nlevels)
{
 
    
    struct HOGDescriptor_t* wrapper  = new HOGDescriptor_t;
try {
    wrapper->v = new cv::HOGDescriptor(*_winSize->v, *_blockSize->v, *_blockStride->v, *_cellSize->v, _nbins, _derivAperture, _winSigma, _histogramNormType, _L2HysThreshold, _gammaCorrection, _nlevels);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  HOGDescriptor_t* )   pCvHOGDescriptorCreate3(string_t* filename)
{
 
    
    struct HOGDescriptor_t* wrapper  = new HOGDescriptor_t;
try {
    wrapper->v = new cv::HOGDescriptor(filename->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvHOGDescriptorDelete(struct HOGDescriptor_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  flann_Index_t* )   pCvflann_IndexCreate()
{
 
    
    struct flann_Index_t* wrapper  = new flann_Index_t;
try {
    wrapper->v = new cv::flann::Index();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  flann_Index_t* )   pCvflann_IndexCreate2(Mat_t* features, IndexParams_t* params, cvflann_flann_distance_t distType)
{
 
    
    struct flann_Index_t* wrapper  = new flann_Index_t;
try {
    wrapper->v = new cv::flann::Index(*features->v, *params->v, distType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvflann_IndexDelete(struct flann_Index_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  KDTree_t* )   pCvKDTreeCreate()
{
 
    
    struct KDTree_t* wrapper  = new KDTree_t;
try {
    wrapper->v = new cv::KDTree();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  KDTree_t* )   pCvKDTreeCreate2(Mat_t* points, bool copyAndReorderPoints)
{
 
    
    struct KDTree_t* wrapper  = new KDTree_t;
try {
    wrapper->v = new cv::KDTree(*points->v, copyAndReorderPoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  KDTree_t* )   pCvKDTreeCreate3(Mat_t* points, Mat_t* _labels, bool copyAndReorderPoints)
{
 
    
    struct KDTree_t* wrapper  = new KDTree_t;
try {
    wrapper->v = new cv::KDTree(*points->v, *_labels->v, copyAndReorderPoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvKDTreeDelete(struct KDTree_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  KalmanFilter_t* )   pCvKalmanFilterCreate()
{
 
    
    struct KalmanFilter_t* wrapper  = new KalmanFilter_t;
try {
    wrapper->v = new cv::KalmanFilter();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  KalmanFilter_t* )   pCvKalmanFilterCreate2(int dynamParams, int measureParams, int controlParams, int _type)
{
 
    
    struct KalmanFilter_t* wrapper  = new KalmanFilter_t;
try {
    wrapper->v = new cv::KalmanFilter(dynamParams, measureParams, controlParams, _type);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvKalmanFilterDelete(struct KalmanFilter_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  KeyPoint_t* )   pCvKeyPointCreate()
{
 
    
    struct KeyPoint_t* wrapper  = new KeyPoint_t;
try {
    wrapper->v = new cv::KeyPoint();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  KeyPoint_t* )   pCvKeyPointCreate2(float x, float y, float _size, float _angle, float _response, int _octave, int _class_id)
{
 
    
    struct KeyPoint_t* wrapper  = new KeyPoint_t;
try {
    wrapper->v = new cv::KeyPoint(x, y, _size, _angle, _response, _octave, _class_id);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvKeyPointDelete(struct KeyPoint_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  MSER_t* )   pCvMSERCreate(int _delta, int _min_area, int _max_area, double _max_variation, double _min_diversity, int _max_evolution, double _area_threshold, double _min_margin, int _edge_blur_size)
{
 
    
    struct MSER_t* wrapper  = new MSER_t;
try {
    wrapper->v = new cv::MSER(_delta, _min_area, _max_area, _max_variation, _min_diversity, _max_evolution, _area_threshold, _min_margin, _edge_blur_size);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvMSERDelete(struct MSER_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  ORB_t* )   pCvORBCreate(int nfeatures, float scaleFactor, int nlevels, int edgeThreshold, int firstLevel, int WTA_K, int scoreType, int patchSize)
{
 
    
    struct ORB_t* wrapper  = new ORB_t;
try {
    wrapper->v = new cv::ORB(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvORBDelete(struct ORB_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  SimpleBlobDetector_Params_t* )   pCvSimpleBlobDetector_ParamsCreate()
{
 
    
    struct SimpleBlobDetector_Params_t* wrapper  = new SimpleBlobDetector_Params_t;
try {
    wrapper->v = new cv::SimpleBlobDetector::Params();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvSimpleBlobDetector_ParamsDelete(struct SimpleBlobDetector_Params_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  PyramidAdaptedFeatureDetector_t* )   pCvPyramidAdaptedFeatureDetectorCreate(Ptr_FeatureDetector* detector, int maxLevel)
{
 
    
    struct PyramidAdaptedFeatureDetector_t* wrapper  = new PyramidAdaptedFeatureDetector_t;
try {
    wrapper->v = new cv::PyramidAdaptedFeatureDetector(*detector, maxLevel);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvPyramidAdaptedFeatureDetectorDelete(struct PyramidAdaptedFeatureDetector_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  SIFT_t* )   pCvSIFTCreate(int nfeatures, int nOctaveLayers, double contrastThreshold, double edgeThreshold, double sigma)
{
 
    
    struct SIFT_t* wrapper  = new SIFT_t;
try {
    wrapper->v = new cv::SIFT(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvSIFTDelete(struct SIFT_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  SURF_t* )   pCvSURFCreate()
{
 
    
    struct SURF_t* wrapper  = new SURF_t;
try {
    wrapper->v = new cv::SURF();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  SURF_t* )   pCvSURFCreate2(double hessianThreshold, int nOctaves, int nOctaveLayers, bool extended, bool upright)
{
 
    
    struct SURF_t* wrapper  = new SURF_t;
try {
    wrapper->v = new cv::SURF(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvSURFDelete(struct SURF_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  SimpleBlobDetector_t* )   pCvSimpleBlobDetectorCreate(SimpleBlobDetector_Params_t* parameters)
{
 
    
    struct SimpleBlobDetector_t* wrapper  = new SimpleBlobDetector_t;
try {
    wrapper->v = new cv::SimpleBlobDetector(*parameters->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvSimpleBlobDetectorDelete(struct SimpleBlobDetector_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  StarDetector_t* )   pCvStarDetectorCreate(int _maxSize, int _responseThreshold, int _lineThresholdProjected, int _lineThresholdBinarized, int _suppressNonmaxSize)
{
 
    
    struct StarDetector_t* wrapper  = new StarDetector_t;
try {
    wrapper->v = new cv::StarDetector(_maxSize, _responseThreshold, _lineThresholdProjected, _lineThresholdBinarized, _suppressNonmaxSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvStarDetectorDelete(struct StarDetector_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  StereoBM_t* )   pCvStereoBMCreate()
{
 
    
    struct StereoBM_t* wrapper  = new StereoBM_t;
try {
    wrapper->v = new cv::StereoBM();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  StereoBM_t* )   pCvStereoBMCreate2(int preset, int ndisparities, int SADWindowSize)
{
 
    
    struct StereoBM_t* wrapper  = new StereoBM_t;
try {
    wrapper->v = new cv::StereoBM(preset, ndisparities, SADWindowSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvStereoBMDelete(struct StereoBM_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  StereoSGBM_t* )   pCvStereoSGBMCreate()
{
 
    
    struct StereoSGBM_t* wrapper  = new StereoSGBM_t;
try {
    wrapper->v = new cv::StereoSGBM();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  StereoSGBM_t* )   pCvStereoSGBMCreate2(int minDisparity, int numDisparities, int SADWindowSize, int P1, int P2, int disp12MaxDiff, int preFilterCap, int uniquenessRatio, int speckleWindowSize, int speckleRange, bool fullDP)
{
 
    
    struct StereoSGBM_t* wrapper  = new StereoSGBM_t;
try {
    wrapper->v = new cv::StereoSGBM(minDisparity, numDisparities, SADWindowSize, P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, fullDP);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvStereoSGBMDelete(struct StereoSGBM_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  StereoVar_t* )   pCvStereoVarCreate()
{
 
    
    struct StereoVar_t* wrapper  = new StereoVar_t;
try {
    wrapper->v = new cv::StereoVar();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  StereoVar_t* )   pCvStereoVarCreate2(int levels, double pyrScale, int nIt, int minDisp, int maxDisp, int poly_n, double poly_sigma, float fi, float lambda, int penalization, int cycle, int flags)
{
 
    
    struct StereoVar_t* wrapper  = new StereoVar_t;
try {
    wrapper->v = new cv::StereoVar(levels, pyrScale, nIt, minDisp, maxDisp, poly_n, poly_sigma, fi, lambda, penalization, cycle, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvStereoVarDelete(struct StereoVar_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  Subdiv2D_t* )   pCvSubdiv2DCreate()
{
 
    
    struct Subdiv2D_t* wrapper  = new Subdiv2D_t;
try {
    wrapper->v = new cv::Subdiv2D();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  Subdiv2D_t* )   pCvSubdiv2DCreate2(Rect_t* rect)
{
 
    
    struct Subdiv2D_t* wrapper  = new Subdiv2D_t;
try {
    wrapper->v = new cv::Subdiv2D(*rect->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvSubdiv2DDelete(struct Subdiv2D_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  VideoCapture_t* )   pCvVideoCaptureCreate()
{
 
    
    struct VideoCapture_t* wrapper  = new VideoCapture_t;
try {
    wrapper->v = new cv::VideoCapture();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  VideoCapture_t* )   pCvVideoCaptureCreate2(string_t* filename)
{
 
    
    struct VideoCapture_t* wrapper  = new VideoCapture_t;
try {
    wrapper->v = new cv::VideoCapture(filename->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  VideoCapture_t* )   pCvVideoCaptureCreate3(int device)
{
 
    
    struct VideoCapture_t* wrapper  = new VideoCapture_t;
try {
    wrapper->v = new cv::VideoCapture(device);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvVideoCaptureDelete(struct VideoCapture_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}
CVAPI(struct  VideoWriter_t* )   pCvVideoWriterCreate()
{
 
    
    struct VideoWriter_t* wrapper  = new VideoWriter_t;
try {
    wrapper->v = new cv::VideoWriter();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}
CVAPI(struct  VideoWriter_t* )   pCvVideoWriterCreate2(string_t* filename, int fourcc, double fps, Size_t* frameSize, bool isColor)
{
 
    
    struct VideoWriter_t* wrapper  = new VideoWriter_t;
try {
    wrapper->v = new cv::VideoWriter(filename->v, fourcc, fps, *frameSize->v, isColor);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (wrapper);

}

CVAPI(void)   pCvVideoWriterDelete(struct VideoWriter_t* wrapper)
{
  assert(wrapper);
  delete wrapper->v;
  delete wrapper;
}

CVAPI(vector_int*) pCvKDTreeGet_labels(KDTree_t* p)
{
    vector_int* retval;
try {

    retval =   &(p->v->labels);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(int) pCvKDTreeGet_maxDepth(KDTree_t* p)
{
    int retval;
try {

    retval =   (p->v->maxDepth);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(int) pCvKDTreeGet_normType(KDTree_t* p)
{
    int retval;
try {

    retval =   (p->v->normType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvKDTreeSet_normType(KDTree_t* p, int val)
{
try {
    p->v->normType = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(Mat_t*) pCvKDTreeGet_points(KDTree_t* p)
{
    Mat_t*  retval = new Mat_t();
try {
    Mat tr =   (p->v->points);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}
CVAPI(void)   pCvKDTreebuild(struct  KDTree_t* wrapper, Mat_t* points, bool copyAndReorderPoints)
{
try {
 
    wrapper->v->build(*points->v, copyAndReorderPoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvKDTreebuild2(struct  KDTree_t* wrapper, Mat_t* points, Mat_t* labels, bool copyAndReorderPoints)
{
try {
 
    wrapper->v->build(*points->v, *labels->v, copyAndReorderPoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvKDTreedims(struct  KDTree_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->dims();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvKDTreefindNearest(struct  KDTree_t* wrapper, Mat_t* vec, int K, int Emax, Mat_t* neighborsIdx, Mat_t* neighbors, Mat_t* dist, Mat_t* labels)
{
    int retval;
try {
 
    retval = wrapper->v->findNearest(*vec->v, K, Emax, *neighborsIdx->v, *neighbors->v, *dist->v, *labels->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvKDTreefindOrthoRange(struct  KDTree_t* wrapper, Mat_t* minBounds, Mat_t* maxBounds, Mat_t* neighborsIdx, Mat_t* neighbors, Mat_t* labels)
{
try {
 
    wrapper->v->findOrthoRange(*minBounds->v, *maxBounds->v, *neighborsIdx->v, *neighbors->v, *labels->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvKDTreegetPoints(struct  KDTree_t* wrapper, Mat_t* idx, Mat_t* pts, Mat_t* labels)
{
try {
 
    wrapper->v->getPoints(*idx->v, *pts->v, *labels->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(FileNode_t*)   pCvFileStoragegetFirstTopLevelNode(struct  FileStorage_t* wrapper)
{
   FileNode_t* retval = new FileNode_t();
try {
 
    retval->v = &wrapper->v->getFirstTopLevelNode();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvFileStorageisOpened(struct  FileStorage_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isOpened();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvFileStorageopen(struct  FileStorage_t* wrapper, string_t* filename, int flags, string_t* encoding)
{
    bool retval;
try {
 
    retval = wrapper->v->open(filename->v, flags, encoding->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(FileNode_t*)   pCvFileStoragegetelem(struct  FileStorage_t* wrapper, c_string nodename)
{
   FileNode_t* retval = new FileNode_t();
try {
 
    retval->v = &wrapper->v->operator[](nodename);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvFileStoragerelease(struct  FileStorage_t* wrapper)
{
try {
 
    wrapper->v->release();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(string_t*)   pCvFileStoragereleaseAndGetString(struct  FileStorage_t* wrapper)
{
    string_t* retval = new string_t();
 try {
 
    const string  sr = wrapper->v->releaseAndGetString();
    int len = sr.length() + 1; 
    retval->v = (char*) cvAlloc(len * sizeof(char)); 
    retval->nrchar = len; 
    strcpy(retval->v, &sr[0]); 
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(FileNode_t*)   pCvFileStorageroot(struct  FileStorage_t* wrapper, int streamidx)
{
   FileNode_t* retval = new FileNode_t();
try {
 
    retval->v = &wrapper->v->root(streamidx);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvFileNodeempty(struct  FileNode_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->empty();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvFileNodeisInt(struct  FileNode_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isInt();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvFileNodeisMap(struct  FileNode_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isMap();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvFileNodeisNamed(struct  FileNode_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isNamed();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvFileNodeisNone(struct  FileNode_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isNone();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvFileNodeisReal(struct  FileNode_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isReal();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvFileNodeisSeq(struct  FileNode_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isSeq();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvFileNodeisString(struct  FileNode_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isString();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(string_t*)   pCvFileNodename(struct  FileNode_t* wrapper)
{
    string_t* retval = new string_t();
 try {
 
    const string  sr = wrapper->v->name();
    int len = sr.length() + 1; 
    retval->v = (char*) cvAlloc(len * sizeof(char)); 
    retval->nrchar = len; 
    strcpy(retval->v, &sr[0]); 
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(FileNode_t*)   pCvFileNodegetelem(struct  FileNode_t* wrapper, c_string nodename)
{
   FileNode_t* retval = new FileNode_t();
try {
 
    retval->v = &wrapper->v->operator[](nodename);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(FileNode_t*)   pCvFileNodegetelem2(struct  FileNode_t* wrapper, int i)
{
   FileNode_t* retval = new FileNode_t();
try {
 
    retval->v = &wrapper->v->operator[](i);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(size_t)   pCvFileNodesize(struct  FileNode_t* wrapper)
{
    size_t retval;
try {
 
    retval = wrapper->v->size();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvFileNodetype(struct  FileNode_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->type();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Ptr_Algorithm*)   pCvAlgorithmgetAlgorithm(struct  Algorithm_t* wrapper, string_t* name)
{
Ptr_Algorithm* retval = 0;
try {
 
    Ptr_Algorithm p = wrapper->v->getAlgorithm(name->v);
    retval = Ptr_cpy(p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvAlgorithmgetBool(struct  Algorithm_t* wrapper, string_t* name)
{
    bool retval;
try {
 
    retval = wrapper->v->getBool(name->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(double)   pCvAlgorithmgetDouble(struct  Algorithm_t* wrapper, string_t* name)
{
    double retval;
try {
 
    retval = wrapper->v->getDouble(name->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvAlgorithmgetInt(struct  Algorithm_t* wrapper, string_t* name)
{
    int retval;
try {
 
    retval = wrapper->v->getInt(name->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvAlgorithmgetMat(struct  Algorithm_t* wrapper, string_t* name)
{
    Mat_t*  retval = new Mat_t();
try { 
    Mat tr = wrapper->v->getMat(name->v);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(vector_Mat*)   pCvAlgorithmgetMatVector(struct  Algorithm_t* wrapper, string_t* name)
{
vector_Mat* retval = new vector_Mat();
try {
 
    vector_Mat tr = wrapper->v->getMatVector(name->v);
    vector_Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvAlgorithmgetParams(struct  Algorithm_t* wrapper, vector_string* names)
{
try {
 
    wrapper->v->getParams(*names);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(string_t*)   pCvAlgorithmgetString(struct  Algorithm_t* wrapper, string_t* name)
{
    string_t* retval = new string_t();
 try {
 
    const string  sr = wrapper->v->getString(name->v);
    int len = sr.length() + 1; 
    retval->v = (char*) cvAlloc(len * sizeof(char)); 
    retval->nrchar = len; 
    strcpy(retval->v, &sr[0]); 
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(string_t*)   pCvAlgorithmparamHelp(struct  Algorithm_t* wrapper, string_t* name)
{
    string_t* retval = new string_t();
 try {
 
    const string  sr = wrapper->v->paramHelp(name->v);
    int len = sr.length() + 1; 
    retval->v = (char*) cvAlloc(len * sizeof(char)); 
    retval->nrchar = len; 
    strcpy(retval->v, &sr[0]); 
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvAlgorithmparamType(struct  Algorithm_t* wrapper, string_t* name)
{
    int retval;
try {
 
    retval = wrapper->v->paramType(name->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvAlgorithmsetAlgorithm(struct  Algorithm_t* wrapper, string_t* name, Ptr_Algorithm* value)
{
try {
 
    wrapper->v->setAlgorithm(name->v, *value);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvAlgorithmsetBool(struct  Algorithm_t* wrapper, string_t* name, bool value)
{
try {
 
    wrapper->v->setBool(name->v, value);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvAlgorithmsetDouble(struct  Algorithm_t* wrapper, string_t* name, double value)
{
try {
 
    wrapper->v->setDouble(name->v, value);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvAlgorithmsetInt(struct  Algorithm_t* wrapper, string_t* name, int value)
{
try {
 
    wrapper->v->setInt(name->v, value);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvAlgorithmsetMat(struct  Algorithm_t* wrapper, string_t* name, Mat_t* value)
{
try {
 
    wrapper->v->setMat(name->v, *value->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvAlgorithmsetMatVector(struct  Algorithm_t* wrapper, string_t* name, vector_Mat* value)
{
try {
 
    wrapper->v->setMatVector(name->v, *value);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvAlgorithmsetString(struct  Algorithm_t* wrapper, string_t* name, string_t* value)
{
try {
 
    wrapper->v->setString(name->v, value->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCvStatModelload(struct  CvStatModel_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->load(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCvStatModelsave(struct  CvStatModel_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->save(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}

CVAPI(double) pCvCvParamGridGet_max_val(CvParamGrid_t* p)
{
    double retval;
try {

    retval =   (p->v->max_val);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvParamGridSet_max_val(CvParamGrid_t* p, double val)
{
try {
    p->v->max_val = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvParamGridGet_min_val(CvParamGrid_t* p)
{
    double retval;
try {

    retval =   (p->v->min_val);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvParamGridSet_min_val(CvParamGrid_t* p, double val)
{
try {
    p->v->min_val = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvParamGridGet_step(CvParamGrid_t* p)
{
    double retval;
try {

    retval =   (p->v->step);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvParamGridSet_step(CvParamGrid_t* p, double val)
{
try {
    p->v->step = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvCvNormalBayesClassifierclear(struct  CvNormalBayesClassifier_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCvNormalBayesClassifierload(struct  CvNormalBayesClassifier_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->load(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(float)   pCvCvNormalBayesClassifierpredict(struct  CvNormalBayesClassifier_t* wrapper, Mat_t* samples, Mat_t* results)
{
    float retval;
try {
 
    retval = wrapper->v->predict(*samples->v, results->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvNormalBayesClassifiersave(struct  CvNormalBayesClassifier_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->save(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvCvNormalBayesClassifiertrain(struct  CvNormalBayesClassifier_t* wrapper, Mat_t* trainData, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, bool update)
{
    bool retval;
try {
 
    retval = wrapper->v->train(*trainData->v, *responses->v, *varIdx->v, *sampleIdx->v, update);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(float)   pCvCvKNearestfind_nearest(struct  CvKNearest_t* wrapper, Mat_t* samples, int k, Mat_t* results, Mat_t* neighborResponses, Mat_t* dists)
{
    float retval;
try {
 
    retval = wrapper->v->find_nearest(*samples->v, k, *results->v, *neighborResponses->v, *dists->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvKNearestload(struct  CvKNearest_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->load(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCvKNearestsave(struct  CvKNearest_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->save(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvCvKNearesttrain(struct  CvKNearest_t* wrapper, Mat_t* trainData, Mat_t* responses, Mat_t* sampleIdx, bool isRegression, int maxK, bool updateBase)
{
    bool retval;
try {
 
    retval = wrapper->v->train(*trainData->v, *responses->v, *sampleIdx->v, isRegression, maxK, updateBase);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}

CVAPI(double) pCvCvSVMParamsGet_C(CvSVMParams_t* p)
{
    double retval;
try {

    retval =   (p->v->C);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvSVMParamsSet_C(CvSVMParams_t* p, double val)
{
try {
    p->v->C = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvSVMParamsGet_coef0(CvSVMParams_t* p)
{
    double retval;
try {

    retval =   (p->v->coef0);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvSVMParamsSet_coef0(CvSVMParams_t* p, double val)
{
try {
    p->v->coef0 = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvSVMParamsGet_degree(CvSVMParams_t* p)
{
    double retval;
try {

    retval =   (p->v->degree);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvSVMParamsSet_degree(CvSVMParams_t* p, double val)
{
try {
    p->v->degree = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvSVMParamsGet_gamma(CvSVMParams_t* p)
{
    double retval;
try {

    retval =   (p->v->gamma);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvSVMParamsSet_gamma(CvSVMParams_t* p, double val)
{
try {
    p->v->gamma = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvCvSVMParamsGet_kernel_type(CvSVMParams_t* p)
{
    int retval;
try {

    retval =   (p->v->kernel_type);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvSVMParamsSet_kernel_type(CvSVMParams_t* p, int val)
{
try {
    p->v->kernel_type = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvSVMParamsGet_nu(CvSVMParams_t* p)
{
    double retval;
try {

    retval =   (p->v->nu);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvSVMParamsSet_nu(CvSVMParams_t* p, double val)
{
try {
    p->v->nu = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvSVMParamsGet_p(CvSVMParams_t* p)
{
    double retval;
try {

    retval =   (p->v->p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvSVMParamsSet_p(CvSVMParams_t* p, double val)
{
try {
    p->v->p = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvCvSVMParamsGet_svm_type(CvSVMParams_t* p)
{
    int retval;
try {

    retval =   (p->v->svm_type);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvSVMParamsSet_svm_type(CvSVMParams_t* p, int val)
{
try {
    p->v->svm_type = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(CvTermCriteria_t*) pCvCvSVMParamsGet_term_crit(CvSVMParams_t* p)
{
    CvTermCriteria_t*  retval = new CvTermCriteria_t();
try {
    CvTermCriteria tr =   (p->v->term_crit);
    CvTermCriteria*  t = new CvTermCriteria();
    retval->v = t;
    CvTermCriteria_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvSVMParamsSet_term_crit(CvSVMParams_t* p, CvTermCriteria_t* val)
{
try {
    p->v->term_crit = *val->v;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvCvSVMclear(struct  CvSVM_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvCvSVMget_support_vector_count(struct  CvSVM_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->get_support_vector_count();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvCvSVMget_var_count(struct  CvSVM_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->get_var_count();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvSVMload(struct  CvSVM_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->load(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(float)   pCvCvSVMpredict(struct  CvSVM_t* wrapper, Mat_t* sample, bool returnDFVal)
{
    float retval;
try {
 
    retval = wrapper->v->predict(*sample->v, returnDFVal);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvSVMpredict_all(struct  CvSVM_t* wrapper, Mat_t* samples, Mat_t* results)
{
try {
 
    wrapper->v->predict(*samples->v, *results->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCvSVMsave(struct  CvSVM_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->save(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvCvSVMtrain(struct  CvSVM_t* wrapper, Mat_t* trainData, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, CvSVMParams_t* params)
{
    bool retval;
try {
 
    retval = wrapper->v->train(*trainData->v, *responses->v, *varIdx->v, *sampleIdx->v, *params->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvCvSVMtrain_auto(struct  CvSVM_t* wrapper, Mat_t* trainData, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, CvSVMParams_t* params, int k_fold, CvParamGrid_t* Cgrid, CvParamGrid_t* gammaGrid, CvParamGrid_t* pGrid, CvParamGrid_t* nuGrid, CvParamGrid_t* coeffGrid, CvParamGrid_t* degreeGrid, bool balanced)
{
    bool retval;
try {
 
    retval = wrapper->v->train_auto(*trainData->v, *responses->v, *varIdx->v, *sampleIdx->v, *params->v, k_fold, *Cgrid->v, *gammaGrid->v, *pGrid->v, *nuGrid->v, *coeffGrid->v, *degreeGrid->v, balanced);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvEMclear(struct  EM_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvEMisTrained(struct  EM_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isTrained();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Vec2d_t*)   pCvEMpredict(struct  EM_t* wrapper, Mat_t* sample, Mat_t* probs)
{
    Vec2d_t*  retval = new Vec2d_t();
try { 
    Vec2d tr = wrapper->v->predict(*sample->v, *probs->v);
    Vec2d*  t = new Vec2d();
    retval->v = t;
    Vec2d_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvEMtrain(struct  EM_t* wrapper, Mat_t* samples, Mat_t* logLikelihoods, Mat_t* labels, Mat_t* probs)
{
    bool retval;
try {
 
    retval = wrapper->v->train(*samples->v, *logLikelihoods->v, *labels->v, *probs->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvEMtrainE(struct  EM_t* wrapper, Mat_t* samples, Mat_t* means0, Mat_t* covs0, Mat_t* weights0, Mat_t* logLikelihoods, Mat_t* labels, Mat_t* probs)
{
    bool retval;
try {
 
    retval = wrapper->v->trainE(*samples->v, *means0->v, *covs0->v, *weights0->v, *logLikelihoods->v, *labels->v, *probs->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvEMtrainM(struct  EM_t* wrapper, Mat_t* samples, Mat_t* probs0, Mat_t* logLikelihoods, Mat_t* labels, Mat_t* probs)
{
    bool retval;
try {
 
    retval = wrapper->v->trainM(*samples->v, *probs0->v, *logLikelihoods->v, *labels->v, *probs->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}

CVAPI(int) pCvCvDTreeParamsGet_cv_folds(CvDTreeParams_t* p)
{
    int retval;
try {

    retval =   (p->v->cv_folds);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvDTreeParamsSet_cv_folds(CvDTreeParams_t* p, int val)
{
try {
    p->v->cv_folds = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvCvDTreeParamsGet_max_categories(CvDTreeParams_t* p)
{
    int retval;
try {

    retval =   (p->v->max_categories);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvDTreeParamsSet_max_categories(CvDTreeParams_t* p, int val)
{
try {
    p->v->max_categories = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvCvDTreeParamsGet_max_depth(CvDTreeParams_t* p)
{
    int retval;
try {

    retval =   (p->v->max_depth);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvDTreeParamsSet_max_depth(CvDTreeParams_t* p, int val)
{
try {
    p->v->max_depth = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvCvDTreeParamsGet_min_sample_count(CvDTreeParams_t* p)
{
    int retval;
try {

    retval =   (p->v->min_sample_count);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvDTreeParamsSet_min_sample_count(CvDTreeParams_t* p, int val)
{
try {
    p->v->min_sample_count = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvCvDTreeParamsGet_regression_accuracy(CvDTreeParams_t* p)
{
    float retval;
try {

    retval =   (p->v->regression_accuracy);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvDTreeParamsSet_regression_accuracy(CvDTreeParams_t* p, float val)
{
try {
    p->v->regression_accuracy = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(bool) pCvCvDTreeParamsGet_truncate_pruned_tree(CvDTreeParams_t* p)
{
    bool retval;
try {

    retval =   (p->v->truncate_pruned_tree);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvDTreeParamsSet_truncate_pruned_tree(CvDTreeParams_t* p, bool val)
{
try {
    p->v->truncate_pruned_tree = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(bool) pCvCvDTreeParamsGet_use_1se_rule(CvDTreeParams_t* p)
{
    bool retval;
try {

    retval =   (p->v->use_1se_rule);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvDTreeParamsSet_use_1se_rule(CvDTreeParams_t* p, bool val)
{
try {
    p->v->use_1se_rule = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(bool) pCvCvDTreeParamsGet_use_surrogates(CvDTreeParams_t* p)
{
    bool retval;
try {

    retval =   (p->v->use_surrogates);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvDTreeParamsSet_use_surrogates(CvDTreeParams_t* p, bool val)
{
try {
    p->v->use_surrogates = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvCvDTreeclear(struct  CvDTree_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvCvDTreegetVarImportance(struct  CvDTree_t* wrapper)
{
    Mat_t*  retval = new Mat_t();
try { 
    Mat tr = wrapper->v->getVarImportance();
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvDTreeload(struct  CvDTree_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->load(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(CvDTreeNode_t*)   pCvCvDTreepredict(struct  CvDTree_t* wrapper, Mat_t* sample, Mat_t* missingDataMask, bool preprocessedInput)
{
    CvDTreeNode_t*  retval = new CvDTreeNode_t();
try { 
    CvDTreeNode* tr = wrapper->v->predict(*sample->v, *missingDataMask->v, preprocessedInput);
    CvDTreeNode*  t = new CvDTreeNode();
    retval->v = t;
    CvDTreeNode_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvDTreesave(struct  CvDTree_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->save(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvCvDTreetrain(struct  CvDTree_t* wrapper, Mat_t* trainData, int tflag, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, Mat_t* varType, Mat_t* missingDataMask, CvDTreeParams_t* params)
{
    bool retval;
try {
 
    retval = wrapper->v->train(*trainData->v, tflag, *responses->v, *varIdx->v, *sampleIdx->v, *varType->v, *missingDataMask->v, *params->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}

CVAPI(bool) pCvCvRTParamsGet_calc_var_importance(CvRTParams_t* p)
{
    bool retval;
try {

    retval =   (p->v->calc_var_importance);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvRTParamsSet_calc_var_importance(CvRTParams_t* p, bool val)
{
try {
    p->v->calc_var_importance = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvCvRTParamsGet_nactive_vars(CvRTParams_t* p)
{
    int retval;
try {

    retval =   (p->v->nactive_vars);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvRTParamsSet_nactive_vars(CvRTParams_t* p, int val)
{
try {
    p->v->nactive_vars = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(CvTermCriteria_t*) pCvCvRTParamsGet_term_crit(CvRTParams_t* p)
{
    CvTermCriteria_t*  retval = new CvTermCriteria_t();
try {
    CvTermCriteria tr =   (p->v->term_crit);
    CvTermCriteria*  t = new CvTermCriteria();
    retval->v = t;
    CvTermCriteria_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvRTParamsSet_term_crit(CvRTParams_t* p, CvTermCriteria_t* val)
{
try {
    p->v->term_crit = *val->v;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvCvRTreesclear(struct  CvRTrees_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvCvRTreesgetVarImportance(struct  CvRTrees_t* wrapper)
{
    Mat_t*  retval = new Mat_t();
try { 
    Mat tr = wrapper->v->getVarImportance();
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvRTreesload(struct  CvRTrees_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->load(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(float)   pCvCvRTreespredict(struct  CvRTrees_t* wrapper, Mat_t* sample, Mat_t* missing)
{
    float retval;
try {
 
    retval = wrapper->v->predict(*sample->v, *missing->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(float)   pCvCvRTreespredict_prob(struct  CvRTrees_t* wrapper, Mat_t* sample, Mat_t* missing)
{
    float retval;
try {
 
    retval = wrapper->v->predict_prob(*sample->v, *missing->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvRTreessave(struct  CvRTrees_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->save(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvCvRTreestrain(struct  CvRTrees_t* wrapper, Mat_t* trainData, int tflag, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, Mat_t* varType, Mat_t* missingDataMask, CvRTParams_t* params)
{
    bool retval;
try {
 
    retval = wrapper->v->train(*trainData->v, tflag, *responses->v, *varIdx->v, *sampleIdx->v, *varType->v, *missingDataMask->v, *params->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvERTreesclear(struct  CvERTrees_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvCvERTreesgetVarImportance(struct  CvERTrees_t* wrapper)
{
    Mat_t*  retval = new Mat_t();
try { 
    Mat tr = wrapper->v->getVarImportance();
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(float)   pCvCvERTreespredict(struct  CvERTrees_t* wrapper, Mat_t* sample, Mat_t* missing)
{
    float retval;
try {
 
    retval = wrapper->v->predict(*sample->v, *missing->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(float)   pCvCvERTreespredict_prob(struct  CvERTrees_t* wrapper, Mat_t* sample, Mat_t* missing)
{
    float retval;
try {
 
    retval = wrapper->v->predict_prob(*sample->v, *missing->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvCvERTreestrain(struct  CvERTrees_t* wrapper, Mat_t* trainData, int tflag, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, Mat_t* varType, Mat_t* missingDataMask, CvRTParams_t* params)
{
    bool retval;
try {
 
    retval = wrapper->v->train(*trainData->v, tflag, *responses->v, *varIdx->v, *sampleIdx->v, *varType->v, *missingDataMask->v, *params->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}

CVAPI(int) pCvCvBoostParamsGet_boost_type(CvBoostParams_t* p)
{
    int retval;
try {

    retval =   (p->v->boost_type);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvBoostParamsSet_boost_type(CvBoostParams_t* p, int val)
{
try {
    p->v->boost_type = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvCvBoostParamsGet_split_criteria(CvBoostParams_t* p)
{
    int retval;
try {

    retval =   (p->v->split_criteria);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvBoostParamsSet_split_criteria(CvBoostParams_t* p, int val)
{
try {
    p->v->split_criteria = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvCvBoostParamsGet_weak_count(CvBoostParams_t* p)
{
    int retval;
try {

    retval =   (p->v->weak_count);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvBoostParamsSet_weak_count(CvBoostParams_t* p, int val)
{
try {
    p->v->weak_count = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvBoostParamsGet_weight_trim_rate(CvBoostParams_t* p)
{
    double retval;
try {

    retval =   (p->v->weight_trim_rate);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvBoostParamsSet_weight_trim_rate(CvBoostParams_t* p, double val)
{
try {
    p->v->weight_trim_rate = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvCvBoostclear(struct  CvBoost_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCvBoostload(struct  CvBoost_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->load(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(float)   pCvCvBoostpredict(struct  CvBoost_t* wrapper, Mat_t* sample, Mat_t* missing, Range_t* slice, bool rawMode, bool returnSum)
{
    float retval;
try {
 
    retval = wrapper->v->predict(*sample->v, *missing->v, *slice->v, rawMode, returnSum);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvBoostprune(struct  CvBoost_t* wrapper, CvSlice_t* slice)
{
try {
 
    wrapper->v->prune(*slice->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCvBoostsave(struct  CvBoost_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->save(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvCvBoosttrain(struct  CvBoost_t* wrapper, Mat_t* trainData, int tflag, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, Mat_t* varType, Mat_t* missingDataMask, CvBoostParams_t* params, bool update)
{
    bool retval;
try {
 
    retval = wrapper->v->train(*trainData->v, tflag, *responses->v, *varIdx->v, *sampleIdx->v, *varType->v, *missingDataMask->v, *params->v, update);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}

CVAPI(int) pCvCvGBTreesParamsGet_loss_function_type(CvGBTreesParams_t* p)
{
    int retval;
try {

    retval =   (p->v->loss_function_type);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvGBTreesParamsSet_loss_function_type(CvGBTreesParams_t* p, int val)
{
try {
    p->v->loss_function_type = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvCvGBTreesParamsGet_shrinkage(CvGBTreesParams_t* p)
{
    float retval;
try {

    retval =   (p->v->shrinkage);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvGBTreesParamsSet_shrinkage(CvGBTreesParams_t* p, float val)
{
try {
    p->v->shrinkage = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvCvGBTreesParamsGet_subsample_portion(CvGBTreesParams_t* p)
{
    float retval;
try {

    retval =   (p->v->subsample_portion);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvGBTreesParamsSet_subsample_portion(CvGBTreesParams_t* p, float val)
{
try {
    p->v->subsample_portion = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvCvGBTreesParamsGet_weak_count(CvGBTreesParams_t* p)
{
    int retval;
try {

    retval =   (p->v->weak_count);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvGBTreesParamsSet_weak_count(CvGBTreesParams_t* p, int val)
{
try {
    p->v->weak_count = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvCvGBTreesclear(struct  CvGBTrees_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCvGBTreesload(struct  CvGBTrees_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->load(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(float)   pCvCvGBTreespredict(struct  CvGBTrees_t* wrapper, Mat_t* sample, Mat_t* missing, Range_t* slice, int k)
{
    float retval;
try {
 
    retval = wrapper->v->predict(*sample->v, *missing->v, *slice->v, k);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvGBTreessave(struct  CvGBTrees_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->save(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvCvGBTreestrain(struct  CvGBTrees_t* wrapper, Mat_t* trainData, int tflag, Mat_t* responses, Mat_t* varIdx, Mat_t* sampleIdx, Mat_t* varType, Mat_t* missingDataMask, CvGBTreesParams_t* params, bool update)
{
    bool retval;
try {
 
    retval = wrapper->v->train(*trainData->v, tflag, *responses->v, *varIdx->v, *sampleIdx->v, *varType->v, *missingDataMask->v, *params->v, update);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}

CVAPI(double) pCvCvANN_MLP_TrainParamsGet_bp_dw_scale(CvANN_MLP_TrainParams_t* p)
{
    double retval;
try {

    retval =   (p->v->bp_dw_scale);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvANN_MLP_TrainParamsSet_bp_dw_scale(CvANN_MLP_TrainParams_t* p, double val)
{
try {
    p->v->bp_dw_scale = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvANN_MLP_TrainParamsGet_bp_moment_scale(CvANN_MLP_TrainParams_t* p)
{
    double retval;
try {

    retval =   (p->v->bp_moment_scale);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvANN_MLP_TrainParamsSet_bp_moment_scale(CvANN_MLP_TrainParams_t* p, double val)
{
try {
    p->v->bp_moment_scale = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvANN_MLP_TrainParamsGet_rp_dw0(CvANN_MLP_TrainParams_t* p)
{
    double retval;
try {

    retval =   (p->v->rp_dw0);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvANN_MLP_TrainParamsSet_rp_dw0(CvANN_MLP_TrainParams_t* p, double val)
{
try {
    p->v->rp_dw0 = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvANN_MLP_TrainParamsGet_rp_dw_max(CvANN_MLP_TrainParams_t* p)
{
    double retval;
try {

    retval =   (p->v->rp_dw_max);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvANN_MLP_TrainParamsSet_rp_dw_max(CvANN_MLP_TrainParams_t* p, double val)
{
try {
    p->v->rp_dw_max = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvANN_MLP_TrainParamsGet_rp_dw_min(CvANN_MLP_TrainParams_t* p)
{
    double retval;
try {

    retval =   (p->v->rp_dw_min);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvANN_MLP_TrainParamsSet_rp_dw_min(CvANN_MLP_TrainParams_t* p, double val)
{
try {
    p->v->rp_dw_min = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvANN_MLP_TrainParamsGet_rp_dw_minus(CvANN_MLP_TrainParams_t* p)
{
    double retval;
try {

    retval =   (p->v->rp_dw_minus);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvANN_MLP_TrainParamsSet_rp_dw_minus(CvANN_MLP_TrainParams_t* p, double val)
{
try {
    p->v->rp_dw_minus = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvCvANN_MLP_TrainParamsGet_rp_dw_plus(CvANN_MLP_TrainParams_t* p)
{
    double retval;
try {

    retval =   (p->v->rp_dw_plus);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvANN_MLP_TrainParamsSet_rp_dw_plus(CvANN_MLP_TrainParams_t* p, double val)
{
try {
    p->v->rp_dw_plus = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(CvTermCriteria_t*) pCvCvANN_MLP_TrainParamsGet_term_crit(CvANN_MLP_TrainParams_t* p)
{
    CvTermCriteria_t*  retval = new CvTermCriteria_t();
try {
    CvTermCriteria tr =   (p->v->term_crit);
    CvTermCriteria*  t = new CvTermCriteria();
    retval->v = t;
    CvTermCriteria_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvANN_MLP_TrainParamsSet_term_crit(CvANN_MLP_TrainParams_t* p, CvTermCriteria_t* val)
{
try {
    p->v->term_crit = *val->v;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvCvANN_MLP_TrainParamsGet_train_method(CvANN_MLP_TrainParams_t* p)
{
    int retval;
try {

    retval =   (p->v->train_method);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvCvANN_MLP_TrainParamsSet_train_method(CvANN_MLP_TrainParams_t* p, int val)
{
try {
    p->v->train_method = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvCvANN_MLPclear(struct  CvANN_MLP_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCvANN_MLP_create(struct  CvANN_MLP_t* wrapper, Mat_t* layerSizes, int activateFunc, double fparam1, double fparam2)
{
try {
 
    wrapper->v->create(*layerSizes->v, activateFunc, fparam1, fparam2);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCvANN_MLPload(struct  CvANN_MLP_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->load(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(float)   pCvCvANN_MLPpredict(struct  CvANN_MLP_t* wrapper, Mat_t* inputs, Mat_t* outputs)
{
    float retval;
try {
 
    retval = wrapper->v->predict(*inputs->v, *outputs->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCvANN_MLPsave(struct  CvANN_MLP_t* wrapper, c_string filename, c_string name)
{
try {
 
    wrapper->v->save(filename, name);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvCvANN_MLPtrain(struct  CvANN_MLP_t* wrapper, Mat_t* inputs, Mat_t* outputs, Mat_t* sampleWeights, Mat_t* sampleIdx, CvANN_MLP_TrainParams_t* params, int flags)
{
    int retval;
try {
 
    retval = wrapper->v->train(*inputs->v, *outputs->v, *sampleWeights->v, *sampleIdx->v, *params->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCLAHEapply(struct  CLAHE_t* wrapper, Mat_t* src, Mat_t* dst)
{
try {
 
    wrapper->v->apply(*src->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCLAHEsetClipLimit(struct  CLAHE_t* wrapper, double clipLimit)
{
try {
 
    wrapper->v->setClipLimit(clipLimit);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCLAHEsetTilesGridSize(struct  CLAHE_t* wrapper, Size_t* tileGridSize)
{
try {
 
    wrapper->v->setTilesGridSize(*tileGridSize->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvSubdiv2DedgeDst(struct  Subdiv2D_t* wrapper, int edge, Point2f_t* dstpt)
{
    int retval;
try {
 
    retval = wrapper->v->edgeDst(edge, dstpt->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvSubdiv2DedgeOrg(struct  Subdiv2D_t* wrapper, int edge, Point2f_t* orgpt)
{
    int retval;
try {
 
    retval = wrapper->v->edgeOrg(edge, orgpt->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvSubdiv2DfindNearest(struct  Subdiv2D_t* wrapper, Point2f_t* pt, Point2f_t* nearestPt)
{
    int retval;
try {
 
    retval = wrapper->v->findNearest(*pt->v, nearestPt->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvSubdiv2DgetEdge(struct  Subdiv2D_t* wrapper, int edge, int nextEdgeType)
{
    int retval;
try {
 
    retval = wrapper->v->getEdge(edge, nextEdgeType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvSubdiv2DgetEdgeList(struct  Subdiv2D_t* wrapper, vector_Vec4f* edgeList)
{
try {
 
    wrapper->v->getEdgeList(*edgeList);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvSubdiv2DgetTriangleList(struct  Subdiv2D_t* wrapper, vector_Vec6f* triangleList)
{
try {
 
    wrapper->v->getTriangleList(*triangleList);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Point2f_t*)   pCvSubdiv2DgetVertex(struct  Subdiv2D_t* wrapper, int vertex, int* firstEdge)
{
    Point2f_t*  retval = new Point2f_t();
try { 
    Point2f tr = wrapper->v->getVertex(vertex, firstEdge);
    Point2f*  t = new Point2f();
    retval->v = t;
    Point2f_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvSubdiv2DgetVoronoiFacetList(struct  Subdiv2D_t* wrapper, vector_int* idx, vector_vector_Point2f* facetList, vector_Point2f* facetCenters)
{
try {
 
    wrapper->v->getVoronoiFacetList(*idx, *facetList, *facetCenters);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvSubdiv2DinitDelaunay(struct  Subdiv2D_t* wrapper, Rect_t* rect)
{
try {
 
    wrapper->v->initDelaunay(*rect->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvSubdiv2Dinsert(struct  Subdiv2D_t* wrapper, Point2f_t* pt)
{
    int retval;
try {
 
    retval = wrapper->v->insert(*pt->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvSubdiv2Dinsert2(struct  Subdiv2D_t* wrapper, vector_Point2f* ptvec)
{
try {
 
    wrapper->v->insert(*ptvec);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvSubdiv2Dlocate(struct  Subdiv2D_t* wrapper, Point2f_t* pt, int edge, int vertex)
{
    int retval;
try {
 
    retval = wrapper->v->locate(*pt->v, edge, vertex);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvSubdiv2DnextEdge(struct  Subdiv2D_t* wrapper, int edge)
{
    int retval;
try {
 
    retval = wrapper->v->nextEdge(edge);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvSubdiv2DrotateEdge(struct  Subdiv2D_t* wrapper, int edge, int rotate)
{
    int retval;
try {
 
    retval = wrapper->v->rotateEdge(edge, rotate);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvSubdiv2DsymEdge(struct  Subdiv2D_t* wrapper, int edge)
{
    int retval;
try {
 
    retval = wrapper->v->symEdge(edge);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvStereoBMcompute(struct  StereoBM_t* wrapper, Mat_t* left, Mat_t* right, Mat_t* disparity, int disptype)
{
try {
 
    wrapper->v->operator ()(*left->v, *right->v, *disparity->v, disptype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}

CVAPI(int) pCvStereoSGBMGet_P1(StereoSGBM_t* p)
{
    int retval;
try {

    retval =   (p->v->P1);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_P1(StereoSGBM_t* p, int val)
{
try {
    p->v->P1 = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoSGBMGet_P2(StereoSGBM_t* p)
{
    int retval;
try {

    retval =   (p->v->P2);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_P2(StereoSGBM_t* p, int val)
{
try {
    p->v->P2 = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoSGBMGet_SADWindowSize(StereoSGBM_t* p)
{
    int retval;
try {

    retval =   (p->v->SADWindowSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_SADWindowSize(StereoSGBM_t* p, int val)
{
try {
    p->v->SADWindowSize = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoSGBMGet_disp12MaxDiff(StereoSGBM_t* p)
{
    int retval;
try {

    retval =   (p->v->disp12MaxDiff);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_disp12MaxDiff(StereoSGBM_t* p, int val)
{
try {
    p->v->disp12MaxDiff = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(bool) pCvStereoSGBMGet_fullDP(StereoSGBM_t* p)
{
    bool retval;
try {

    retval =   (p->v->fullDP);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_fullDP(StereoSGBM_t* p, bool val)
{
try {
    p->v->fullDP = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoSGBMGet_minDisparity(StereoSGBM_t* p)
{
    int retval;
try {

    retval =   (p->v->minDisparity);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_minDisparity(StereoSGBM_t* p, int val)
{
try {
    p->v->minDisparity = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoSGBMGet_numberOfDisparities(StereoSGBM_t* p)
{
    int retval;
try {

    retval =   (p->v->numberOfDisparities);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_numberOfDisparities(StereoSGBM_t* p, int val)
{
try {
    p->v->numberOfDisparities = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoSGBMGet_preFilterCap(StereoSGBM_t* p)
{
    int retval;
try {

    retval =   (p->v->preFilterCap);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_preFilterCap(StereoSGBM_t* p, int val)
{
try {
    p->v->preFilterCap = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoSGBMGet_speckleRange(StereoSGBM_t* p)
{
    int retval;
try {

    retval =   (p->v->speckleRange);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_speckleRange(StereoSGBM_t* p, int val)
{
try {
    p->v->speckleRange = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoSGBMGet_speckleWindowSize(StereoSGBM_t* p)
{
    int retval;
try {

    retval =   (p->v->speckleWindowSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_speckleWindowSize(StereoSGBM_t* p, int val)
{
try {
    p->v->speckleWindowSize = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoSGBMGet_uniquenessRatio(StereoSGBM_t* p)
{
    int retval;
try {

    retval =   (p->v->uniquenessRatio);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoSGBMSet_uniquenessRatio(StereoSGBM_t* p, int val)
{
try {
    p->v->uniquenessRatio = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvStereoSGBMcompute(struct  StereoSGBM_t* wrapper, Mat_t* left, Mat_t* right, Mat_t* disp)
{
try {
 
    wrapper->v->operator ()(*left->v, *right->v, *disp->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}

CVAPI(float) pCvKeyPointGet_angle(KeyPoint_t* p)
{
    float retval;
try {

    retval =   (p->v->angle);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvKeyPointSet_angle(KeyPoint_t* p, float val)
{
try {
    p->v->angle = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvKeyPointGet_class_id(KeyPoint_t* p)
{
    int retval;
try {

    retval =   (p->v->class_id);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvKeyPointSet_class_id(KeyPoint_t* p, int val)
{
try {
    p->v->class_id = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvKeyPointGet_octave(KeyPoint_t* p)
{
    int retval;
try {

    retval =   (p->v->octave);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvKeyPointSet_octave(KeyPoint_t* p, int val)
{
try {
    p->v->octave = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(Point2f_t*) pCvKeyPointGet_pt(KeyPoint_t* p)
{
    Point2f_t*  retval = new Point2f_t();
try {
    Point2f tr =   (p->v->pt);
    Point2f*  t = new Point2f();
    retval->v = t;
    Point2f_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvKeyPointSet_pt(KeyPoint_t* p, Point2f_t* val)
{
try {
    p->v->pt = *val->v;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvKeyPointGet_response(KeyPoint_t* p)
{
    float retval;
try {

    retval =   (p->v->response);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvKeyPointSet_response(KeyPoint_t* p, float val)
{
try {
    p->v->response = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvKeyPointGet_size(KeyPoint_t* p)
{
    float retval;
try {

    retval =   (p->v->size);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvKeyPointSet_size(KeyPoint_t* p, float val)
{
try {
    p->v->size = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvFeatureDetectordetect(struct  FeatureDetector_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* mask)
{
try {
 
    wrapper->v->detect(*image->v, *keypoints, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvFeatureDetectorempty(struct  FeatureDetector_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->empty();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvDescriptorExtractorcompute(struct  DescriptorExtractor_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* descriptors)
{
try {
 
    wrapper->v->compute(*image->v, *keypoints, *descriptors->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvDescriptorExtractorempty(struct  DescriptorExtractor_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->empty();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvFeature2Dcompute(struct  Feature2D_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* descriptors)
{
try {
 
    wrapper->v->compute(*image->v, *keypoints, *descriptors->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFeature2Ddetect(struct  Feature2D_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* mask)
{
try {
 
    wrapper->v->detect(*image->v, *keypoints, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFeature2DdetectAndCompute(struct  Feature2D_t* wrapper, Mat_t* image, Mat_t* mask, vector_KeyPoint* keypoints, Mat_t* descriptors, bool useProvidedKeypoints)
{
try {
 
    wrapper->v->operator ()(*image->v, *mask->v, *keypoints, *descriptors->v, useProvidedKeypoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBRISKcompute(struct  BRISK_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* descriptors)
{
try {
 
    wrapper->v->compute(*image->v, *keypoints, *descriptors->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBRISKdetectAndCompute(struct  BRISK_t* wrapper, Mat_t* image, Mat_t* mask, vector_KeyPoint* keypoints, Mat_t* descriptors, bool useProvidedKeypoints)
{
try {
 
    wrapper->v->operator ()(*image->v, *mask->v, *keypoints, *descriptors->v, useProvidedKeypoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBRISKgenerateKernel(struct  BRISK_t* wrapper, vector_float* radiusList, vector_int* numberList, float dMax, float dMin, vector_int* indexChange)
{
try {
 
    wrapper->v->generateKernel(*radiusList, *numberList, dMax, dMin, *indexChange);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvORBcompute(struct  ORB_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* descriptors)
{
try {
 
    wrapper->v->compute(*image->v, *keypoints, *descriptors->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvORBdetect(struct  ORB_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* mask)
{
try {
 
    wrapper->v->detect(*image->v, *keypoints, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvORBdetectAndCompute(struct  ORB_t* wrapper, Mat_t* image, Mat_t* mask, vector_KeyPoint* keypoints, Mat_t* descriptors, bool useProvidedKeypoints)
{
try {
 
    wrapper->v->operator ()(*image->v, *mask->v, *keypoints, *descriptors->v, useProvidedKeypoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvMSERdetect(struct  MSER_t* wrapper, Mat_t* image, vector_vector_Point* msers, Mat_t* mask)
{
try {
 
    wrapper->v->operator ()(*image->v, *msers, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvStarDetectordetect(struct  StarDetector_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints)
{
try {
 
    wrapper->v->operator ()(*image->v, *keypoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFastFeatureDetectordetect(struct  FastFeatureDetector_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* mask)
{
try {
 
    wrapper->v->detect(*image->v, *keypoints, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvGFTTDetectordetect(struct  GFTTDetector_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* mask)
{
try {
 
    wrapper->v->detect(*image->v, *keypoints, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvSimpleBlobDetectordetect(struct  SimpleBlobDetector_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* mask)
{
try {
 
    wrapper->v->detect(*image->v, *keypoints, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}

CVAPI(uchar) pCvSimpleBlobDetector_ParamsGet_blobColor(SimpleBlobDetector_Params_t* p)
{
    uchar retval;
try {

    retval =   (p->v->blobColor);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_blobColor(SimpleBlobDetector_Params_t* p, uchar val)
{
try {
    p->v->blobColor = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(bool) pCvSimpleBlobDetector_ParamsGet_filterByArea(SimpleBlobDetector_Params_t* p)
{
    bool retval;
try {

    retval =   (p->v->filterByArea);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_filterByArea(SimpleBlobDetector_Params_t* p, bool val)
{
try {
    p->v->filterByArea = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(bool) pCvSimpleBlobDetector_ParamsGet_filterByCircularity(SimpleBlobDetector_Params_t* p)
{
    bool retval;
try {

    retval =   (p->v->filterByCircularity);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_filterByCircularity(SimpleBlobDetector_Params_t* p, bool val)
{
try {
    p->v->filterByCircularity = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(bool) pCvSimpleBlobDetector_ParamsGet_filterByColor(SimpleBlobDetector_Params_t* p)
{
    bool retval;
try {

    retval =   (p->v->filterByColor);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_filterByColor(SimpleBlobDetector_Params_t* p, bool val)
{
try {
    p->v->filterByColor = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(bool) pCvSimpleBlobDetector_ParamsGet_filterByConvexity(SimpleBlobDetector_Params_t* p)
{
    bool retval;
try {

    retval =   (p->v->filterByConvexity);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_filterByConvexity(SimpleBlobDetector_Params_t* p, bool val)
{
try {
    p->v->filterByConvexity = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(bool) pCvSimpleBlobDetector_ParamsGet_filterByInertia(SimpleBlobDetector_Params_t* p)
{
    bool retval;
try {

    retval =   (p->v->filterByInertia);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_filterByInertia(SimpleBlobDetector_Params_t* p, bool val)
{
try {
    p->v->filterByInertia = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_maxArea(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->maxArea);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_maxArea(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->maxArea = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_maxCircularity(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->maxCircularity);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_maxCircularity(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->maxCircularity = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_maxConvexity(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->maxConvexity);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_maxConvexity(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->maxConvexity = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_maxInertiaRatio(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->maxInertiaRatio);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_maxInertiaRatio(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->maxInertiaRatio = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_maxThreshold(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->maxThreshold);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_maxThreshold(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->maxThreshold = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_minArea(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->minArea);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_minArea(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->minArea = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_minCircularity(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->minCircularity);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_minCircularity(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->minCircularity = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_minConvexity(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->minConvexity);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_minConvexity(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->minConvexity = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_minDistBetweenBlobs(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->minDistBetweenBlobs);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_minDistBetweenBlobs(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->minDistBetweenBlobs = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_minInertiaRatio(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->minInertiaRatio);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_minInertiaRatio(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->minInertiaRatio = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(size_t) pCvSimpleBlobDetector_ParamsGet_minRepeatability(SimpleBlobDetector_Params_t* p)
{
    size_t retval;
try {

    retval =   (p->v->minRepeatability);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_minRepeatability(SimpleBlobDetector_Params_t* p, size_t val)
{
try {
    p->v->minRepeatability = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_minThreshold(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->minThreshold);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_minThreshold(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->minThreshold = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvSimpleBlobDetector_ParamsGet_thresholdStep(SimpleBlobDetector_Params_t* p)
{
    float retval;
try {

    retval =   (p->v->thresholdStep);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSimpleBlobDetector_ParamsSet_thresholdStep(SimpleBlobDetector_Params_t* p, float val)
{
try {
    p->v->thresholdStep = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvGridAdaptedFeatureDetectordetect(struct  GridAdaptedFeatureDetector_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* mask)
{
try {
 
    wrapper->v->detect(*image->v, *keypoints, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvPyramidAdaptedFeatureDetectordetect(struct  PyramidAdaptedFeatureDetector_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* mask)
{
try {
 
    wrapper->v->detect(*image->v, *keypoints, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}

CVAPI(float) pCvDMatchGet_distance(DMatch_t* p)
{
    float retval;
try {

    retval =   (p->v->distance);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvDMatchSet_distance(DMatch_t* p, float val)
{
try {
    p->v->distance = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvDMatchGet_imgIdx(DMatch_t* p)
{
    int retval;
try {

    retval =   (p->v->imgIdx);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvDMatchSet_imgIdx(DMatch_t* p, int val)
{
try {
    p->v->imgIdx = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvDMatchGet_queryIdx(DMatch_t* p)
{
    int retval;
try {

    retval =   (p->v->queryIdx);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvDMatchSet_queryIdx(DMatch_t* p, int val)
{
try {
    p->v->queryIdx = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvDMatchGet_trainIdx(DMatch_t* p)
{
    int retval;
try {

    retval =   (p->v->trainIdx);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvDMatchSet_trainIdx(DMatch_t* p, int val)
{
try {
    p->v->trainIdx = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvDescriptorMatcheradd(struct  DescriptorMatcher_t* wrapper, vector_Mat* descriptors)
{
try {
 
    wrapper->v->add(*descriptors);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvDescriptorMatcherclear(struct  DescriptorMatcher_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvDescriptorMatcherempty(struct  DescriptorMatcher_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->empty();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(vector_Mat*)   pCvDescriptorMatchergetTrainDescriptors(struct  DescriptorMatcher_t* wrapper)
{
vector_Mat* retval = new vector_Mat();
try {
 
    vector_Mat tr = wrapper->v->getTrainDescriptors();
    vector_Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvDescriptorMatcherknnMatch(struct  DescriptorMatcher_t* wrapper, Mat_t* queryDescriptors, Mat_t* trainDescriptors, vector_vector_DMatch* matches, int k, Mat_t* mask, bool compactResult)
{
try {
 
    wrapper->v->knnMatch(*queryDescriptors->v, *trainDescriptors->v, *matches, k, *mask->v, compactResult);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvDescriptorMatcherknnMatch2(struct  DescriptorMatcher_t* wrapper, Mat_t* queryDescriptors, vector_vector_DMatch* matches, int k, vector_Mat* masks, bool compactResult)
{
try {
 
    wrapper->v->knnMatch(*queryDescriptors->v, *matches, k, *masks, compactResult);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvDescriptorMatchermatch(struct  DescriptorMatcher_t* wrapper, Mat_t* queryDescriptors, Mat_t* trainDescriptors, vector_DMatch* matches, Mat_t* mask)
{
try {
 
    wrapper->v->match(*queryDescriptors->v, *trainDescriptors->v, *matches, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvDescriptorMatchermatch2(struct  DescriptorMatcher_t* wrapper, Mat_t* queryDescriptors, vector_DMatch* matches, vector_Mat* masks)
{
try {
 
    wrapper->v->match(*queryDescriptors->v, *matches, *masks);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvDescriptorMatchertrain(struct  DescriptorMatcher_t* wrapper)
{
try {
 
    wrapper->v->train();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBFMatcheradd(struct  BFMatcher_t* wrapper, vector_Mat* descriptors)
{
try {
 
    wrapper->v->add(*descriptors);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBFMatcherclear(struct  BFMatcher_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvBFMatcherempty(struct  BFMatcher_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->empty();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(vector_Mat*)   pCvBFMatchergetTrainDescriptors(struct  BFMatcher_t* wrapper)
{
vector_Mat* retval = new vector_Mat();
try {
 
    vector_Mat tr = wrapper->v->getTrainDescriptors();
    vector_Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvBFMatcherknnMatch(struct  BFMatcher_t* wrapper, Mat_t* queryDescriptors, Mat_t* trainDescriptors, vector_vector_DMatch* matches, int k, Mat_t* mask, bool compactResult)
{
try {
 
    wrapper->v->knnMatch(*queryDescriptors->v, *trainDescriptors->v, *matches, k, *mask->v, compactResult);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBFMatcherknnMatch2(struct  BFMatcher_t* wrapper, Mat_t* queryDescriptors, vector_vector_DMatch* matches, int k, vector_Mat* masks, bool compactResult)
{
try {
 
    wrapper->v->knnMatch(*queryDescriptors->v, *matches, k, *masks, compactResult);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBFMatchermatch(struct  BFMatcher_t* wrapper, Mat_t* queryDescriptors, Mat_t* trainDescriptors, vector_DMatch* matches, Mat_t* mask)
{
try {
 
    wrapper->v->match(*queryDescriptors->v, *trainDescriptors->v, *matches, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBFMatchermatch2(struct  BFMatcher_t* wrapper, Mat_t* queryDescriptors, vector_DMatch* matches, vector_Mat* masks)
{
try {
 
    wrapper->v->match(*queryDescriptors->v, *matches, *masks);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBFMatchertrain(struct  BFMatcher_t* wrapper)
{
try {
 
    wrapper->v->train();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFlannBasedMatcheradd(struct  FlannBasedMatcher_t* wrapper, vector_Mat* descriptors)
{
try {
 
    wrapper->v->add(*descriptors);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFlannBasedMatcherclear(struct  FlannBasedMatcher_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvFlannBasedMatcherempty(struct  FlannBasedMatcher_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->empty();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(vector_Mat*)   pCvFlannBasedMatchergetTrainDescriptors(struct  FlannBasedMatcher_t* wrapper)
{
vector_Mat* retval = new vector_Mat();
try {
 
    vector_Mat tr = wrapper->v->getTrainDescriptors();
    vector_Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvFlannBasedMatcherknnMatch(struct  FlannBasedMatcher_t* wrapper, Mat_t* queryDescriptors, Mat_t* trainDescriptors, vector_vector_DMatch* matches, int k, Mat_t* mask, bool compactResult)
{
try {
 
    wrapper->v->knnMatch(*queryDescriptors->v, *trainDescriptors->v, *matches, k, *mask->v, compactResult);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFlannBasedMatcherknnMatch2(struct  FlannBasedMatcher_t* wrapper, Mat_t* queryDescriptors, vector_vector_DMatch* matches, int k, vector_Mat* masks, bool compactResult)
{
try {
 
    wrapper->v->knnMatch(*queryDescriptors->v, *matches, k, *masks, compactResult);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFlannBasedMatchermatch(struct  FlannBasedMatcher_t* wrapper, Mat_t* queryDescriptors, Mat_t* trainDescriptors, vector_DMatch* matches, Mat_t* mask)
{
try {
 
    wrapper->v->match(*queryDescriptors->v, *trainDescriptors->v, *matches, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFlannBasedMatchermatch2(struct  FlannBasedMatcher_t* wrapper, Mat_t* queryDescriptors, vector_DMatch* matches, vector_Mat* masks)
{
try {
 
    wrapper->v->match(*queryDescriptors->v, *matches, *masks);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFlannBasedMatchertrain(struct  FlannBasedMatcher_t* wrapper)
{
try {
 
    wrapper->v->train();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBOWTraineradd(struct  BOWTrainer_t* wrapper, Mat_t* descriptors)
{
try {
 
    wrapper->v->add(*descriptors->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBOWTrainerclear(struct  BOWTrainer_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvBOWTrainerdescripotorsCount(struct  BOWTrainer_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->descripotorsCount();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(vector_Mat*)   pCvBOWTrainergetDescriptors(struct  BOWTrainer_t* wrapper)
{
vector_Mat* retval = new vector_Mat();
try {
 
    vector_Mat tr = wrapper->v->getDescriptors();
    vector_Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvBOWKMeansTraineradd(struct  BOWKMeansTrainer_t* wrapper, Mat_t* descriptors)
{
try {
 
    wrapper->v->add(*descriptors->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBOWKMeansTrainerclear(struct  BOWKMeansTrainer_t* wrapper)
{
try {
 
    wrapper->v->clear();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvBOWKMeansTrainercluster(struct  BOWKMeansTrainer_t* wrapper)
{
    Mat_t*  retval = new Mat_t();
try { 
    Mat tr = wrapper->v->cluster();
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvBOWKMeansTrainercluster2(struct  BOWKMeansTrainer_t* wrapper, Mat_t* descriptors)
{
    Mat_t*  retval = new Mat_t();
try { 
    Mat tr = wrapper->v->cluster(*descriptors->v);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvBOWKMeansTrainerdescripotorsCount(struct  BOWKMeansTrainer_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->descripotorsCount();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(vector_Mat*)   pCvBOWKMeansTrainergetDescriptors(struct  BOWKMeansTrainer_t* wrapper)
{
vector_Mat* retval = new vector_Mat();
try {
 
    vector_Mat tr = wrapper->v->getDescriptors();
    vector_Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvBOWImgDescriptorExtractorcompute(struct  BOWImgDescriptorExtractor_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* imgDescriptor)
{
try {
 
    wrapper->v->compute2(*image->v, *keypoints, *imgDescriptor->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvBOWImgDescriptorExtractordescriptorSize(struct  BOWImgDescriptorExtractor_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->descriptorSize();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvBOWImgDescriptorExtractordescriptorType(struct  BOWImgDescriptorExtractor_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->descriptorType();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvBOWImgDescriptorExtractorgetVocabulary(struct  BOWImgDescriptorExtractor_t* wrapper)
{
    Mat_t*  retval = new Mat_t();
try { 
    Mat tr = wrapper->v->getVocabulary();
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvBOWImgDescriptorExtractorsetVocabulary(struct  BOWImgDescriptorExtractor_t* wrapper, Mat_t* vocabulary)
{
try {
 
    wrapper->v->setVocabulary(*vocabulary->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvKalmanFiltercorrect(struct  KalmanFilter_t* wrapper, Mat_t* measurement)
{
    Mat_t*  retval = new Mat_t();
try { 
    Mat tr = wrapper->v->correct(*measurement->v);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvKalmanFilterpredict(struct  KalmanFilter_t* wrapper, Mat_t* control)
{
    Mat_t*  retval = new Mat_t();
try { 
    Mat tr = wrapper->v->predict(*control->v);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvBackgroundSubtractorapply(struct  BackgroundSubtractor_t* wrapper, Mat_t* image, Mat_t* fgmask, double learningRate)
{
try {
 
    wrapper->v->operator ()(*image->v, *fgmask->v, learningRate);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBackgroundSubtractorMOGapply(struct  BackgroundSubtractorMOG_t* wrapper, Mat_t* image, Mat_t* fgmask, double learningRate)
{
try {
 
    wrapper->v->operator ()(*image->v, *fgmask->v, learningRate);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvBackgroundSubtractorMOG2apply(struct  BackgroundSubtractorMOG2_t* wrapper, Mat_t* image, Mat_t* fgmask, double learningRate)
{
try {
 
    wrapper->v->operator ()(*image->v, *fgmask->v, learningRate);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCascadeClassifierdetectMultiScale(struct  CascadeClassifier_t* wrapper, Mat_t* image, vector_Rect* objects, double scaleFactor, int minNeighbors, int flags, Size_t* minSize, Size_t* maxSize)
{
try {
 
    wrapper->v->detectMultiScale(*image->v, *objects, scaleFactor, minNeighbors, flags, *minSize->v, *maxSize->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvCascadeClassifierdetectMultiScale2(struct  CascadeClassifier_t* wrapper, Mat_t* image, vector_Rect* objects, vector_int* rejectLevels, vector_double* levelWeights, double scaleFactor, int minNeighbors, int flags, Size_t* minSize, Size_t* maxSize, bool outputRejectLevels)
{
try {
 
    wrapper->v->detectMultiScale(*image->v, *objects, *rejectLevels, *levelWeights, scaleFactor, minNeighbors, flags, *minSize->v, *maxSize->v, outputRejectLevels);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvCascadeClassifierempty(struct  CascadeClassifier_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->empty();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvCascadeClassifierload(struct  CascadeClassifier_t* wrapper, string_t* filename)
{
    bool retval;
try {
 
    retval = wrapper->v->load(filename->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}

CVAPI(double) pCvHOGDescriptorGet_L2HysThreshold(HOGDescriptor_t* p)
{
    double retval;
try {

    retval =   (p->v->L2HysThreshold);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(Size_t*) pCvHOGDescriptorGet_blockSize(HOGDescriptor_t* p)
{
    Size_t*  retval = new Size_t();
try {
    Size tr =   (p->v->blockSize);
    Size*  t = new Size();
    retval->v = t;
    Size_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(Size_t*) pCvHOGDescriptorGet_blockStride(HOGDescriptor_t* p)
{
    Size_t*  retval = new Size_t();
try {
    Size tr =   (p->v->blockStride);
    Size*  t = new Size();
    retval->v = t;
    Size_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(Size_t*) pCvHOGDescriptorGet_cellSize(HOGDescriptor_t* p)
{
    Size_t*  retval = new Size_t();
try {
    Size tr =   (p->v->cellSize);
    Size*  t = new Size();
    retval->v = t;
    Size_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(int) pCvHOGDescriptorGet_derivAperture(HOGDescriptor_t* p)
{
    int retval;
try {

    retval =   (p->v->derivAperture);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(bool) pCvHOGDescriptorGet_gammaCorrection(HOGDescriptor_t* p)
{
    bool retval;
try {

    retval =   (p->v->gammaCorrection);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(int) pCvHOGDescriptorGet_histogramNormType(HOGDescriptor_t* p)
{
    int retval;
try {

    retval =   (p->v->histogramNormType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(int) pCvHOGDescriptorGet_nbins(HOGDescriptor_t* p)
{
    int retval;
try {

    retval =   (p->v->nbins);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(int) pCvHOGDescriptorGet_nlevels(HOGDescriptor_t* p)
{
    int retval;
try {

    retval =   (p->v->nlevels);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(vector_float*) pCvHOGDescriptorGet_svmDetector(HOGDescriptor_t* p)
{
    vector_float* retval;
try {

    retval =   &(p->v->svmDetector);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(double) pCvHOGDescriptorGet_winSigma(HOGDescriptor_t* p)
{
    double retval;
try {

    retval =   (p->v->winSigma);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(Size_t*) pCvHOGDescriptorGet_winSize(HOGDescriptor_t* p)
{
    Size_t*  retval = new Size_t();
try {
    Size tr =   (p->v->winSize);
    Size*  t = new Size();
    retval->v = t;
    Size_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}
CVAPI(bool)   pCvHOGDescriptorcheckDetectorSize(struct  HOGDescriptor_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->checkDetectorSize();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvHOGDescriptorcompute(struct  HOGDescriptor_t* wrapper, Mat_t* img, vector_float* descriptors, Size_t* winStride, Size_t* padding, vector_Point* locations)
{
try {
 
    wrapper->v->compute(*img->v, *descriptors, *winStride->v, *padding->v, *locations);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvHOGDescriptorcomputeGradient(struct  HOGDescriptor_t* wrapper, Mat_t* img, Mat_t* grad, Mat_t* angleOfs, Size_t* paddingTL, Size_t* paddingBR)
{
try {
 
    wrapper->v->computeGradient(*img->v, *grad->v, *angleOfs->v, *paddingTL->v, *paddingBR->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvHOGDescriptordetect(struct  HOGDescriptor_t* wrapper, Mat_t* img, vector_Point* foundLocations, vector_double* weights, double hitThreshold, Size_t* winStride, Size_t* padding, vector_Point* searchLocations)
{
try {
 
    wrapper->v->detect(*img->v, *foundLocations, *weights, hitThreshold, *winStride->v, *padding->v, *searchLocations);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvHOGDescriptordetectMultiScale(struct  HOGDescriptor_t* wrapper, Mat_t* img, vector_Rect* foundLocations, vector_double* foundWeights, double hitThreshold, Size_t* winStride, Size_t* padding, double scale, double finalThreshold, bool useMeanshiftGrouping)
{
try {
 
    wrapper->v->detectMultiScale(*img->v, *foundLocations, *foundWeights, hitThreshold, *winStride->v, *padding->v, scale, finalThreshold, useMeanshiftGrouping);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(size_t)   pCvHOGDescriptorgetDescriptorSize(struct  HOGDescriptor_t* wrapper)
{
    size_t retval;
try {
 
    retval = wrapper->v->getDescriptorSize();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(double)   pCvHOGDescriptorgetWinSigma(struct  HOGDescriptor_t* wrapper)
{
    double retval;
try {
 
    retval = wrapper->v->getWinSigma();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvHOGDescriptorload(struct  HOGDescriptor_t* wrapper, string_t* filename, string_t* objname)
{
    bool retval;
try {
 
    retval = wrapper->v->load(filename->v, objname->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvHOGDescriptorsave(struct  HOGDescriptor_t* wrapper, string_t* filename, string_t* objname)
{
try {
 
    wrapper->v->save(filename->v, objname->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvHOGDescriptorsetSVMDetector(struct  HOGDescriptor_t* wrapper, Mat_t* _svmdetector)
{
try {
 
    wrapper->v->setSVMDetector(*_svmdetector->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}

CVAPI(int) pCvStereoVarGet_cycle(StereoVar_t* p)
{
    int retval;
try {

    retval =   (p->v->cycle);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_cycle(StereoVar_t* p, int val)
{
try {
    p->v->cycle = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvStereoVarGet_fi(StereoVar_t* p)
{
    float retval;
try {

    retval =   (p->v->fi);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_fi(StereoVar_t* p, float val)
{
try {
    p->v->fi = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoVarGet_flags(StereoVar_t* p)
{
    int retval;
try {

    retval =   (p->v->flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_flags(StereoVar_t* p, int val)
{
try {
    p->v->flags = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(float) pCvStereoVarGet_lambda(StereoVar_t* p)
{
    float retval;
try {

    retval =   (p->v->lambda);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_lambda(StereoVar_t* p, float val)
{
try {
    p->v->lambda = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoVarGet_levels(StereoVar_t* p)
{
    int retval;
try {

    retval =   (p->v->levels);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_levels(StereoVar_t* p, int val)
{
try {
    p->v->levels = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoVarGet_maxDisp(StereoVar_t* p)
{
    int retval;
try {

    retval =   (p->v->maxDisp);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_maxDisp(StereoVar_t* p, int val)
{
try {
    p->v->maxDisp = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoVarGet_minDisp(StereoVar_t* p)
{
    int retval;
try {

    retval =   (p->v->minDisp);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_minDisp(StereoVar_t* p, int val)
{
try {
    p->v->minDisp = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoVarGet_nIt(StereoVar_t* p)
{
    int retval;
try {

    retval =   (p->v->nIt);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_nIt(StereoVar_t* p, int val)
{
try {
    p->v->nIt = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoVarGet_penalization(StereoVar_t* p)
{
    int retval;
try {

    retval =   (p->v->penalization);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_penalization(StereoVar_t* p, int val)
{
try {
    p->v->penalization = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvStereoVarGet_poly_n(StereoVar_t* p)
{
    int retval;
try {

    retval =   (p->v->poly_n);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_poly_n(StereoVar_t* p, int val)
{
try {
    p->v->poly_n = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvStereoVarGet_poly_sigma(StereoVar_t* p)
{
    double retval;
try {

    retval =   (p->v->poly_sigma);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_poly_sigma(StereoVar_t* p, double val)
{
try {
    p->v->poly_sigma = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvStereoVarGet_pyrScale(StereoVar_t* p)
{
    double retval;
try {

    retval =   (p->v->pyrScale);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvStereoVarSet_pyrScale(StereoVar_t* p, double val)
{
try {
    p->v->pyrScale = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvStereoVarcompute(struct  StereoVar_t* wrapper, Mat_t* left, Mat_t* right, Mat_t* disp)
{
try {
 
    wrapper->v->operator ()(*left->v, *right->v, *disp->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFaceRecognizerload(struct  FaceRecognizer_t* wrapper, string_t* filename)
{
try {
 
    wrapper->v->load(filename->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFaceRecognizerpredict(struct  FaceRecognizer_t* wrapper, Mat_t* src, int _label, double confidence)
{
try {
 
    wrapper->v->predict(*src->v, _label, confidence);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFaceRecognizersave(struct  FaceRecognizer_t* wrapper, string_t* filename)
{
try {
 
    wrapper->v->save(filename->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFaceRecognizertrain(struct  FaceRecognizer_t* wrapper, vector_Mat* src, Mat_t* labels)
{
try {
 
    wrapper->v->train(*src, *labels->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvFaceRecognizerupdate(struct  FaceRecognizer_t* wrapper, vector_Mat* src, Mat_t* labels)
{
try {
 
    wrapper->v->update(*src, *labels->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvVideoCaptureget(struct  VideoCapture_t* wrapper, int propId)
{
    double retval;
try {
 
    retval = wrapper->v->get(propId);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvVideoCapturegrab(struct  VideoCapture_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->grab();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvVideoCaptureisOpened(struct  VideoCapture_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isOpened();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvVideoCaptureopen(struct  VideoCapture_t* wrapper, string_t* filename)
{
    bool retval;
try {
 
    retval = wrapper->v->open(filename->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvVideoCaptureopen2(struct  VideoCapture_t* wrapper, int device)
{
    bool retval;
try {
 
    retval = wrapper->v->open(device);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvVideoCaptureread(struct  VideoCapture_t* wrapper, Mat_t* image)
{
    bool retval;
try {
 
    retval = wrapper->v->read(*image->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvVideoCapturerelease(struct  VideoCapture_t* wrapper)
{
try {
 
    wrapper->v->release();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvVideoCaptureretrieve(struct  VideoCapture_t* wrapper, Mat_t* image, int channel)
{
    bool retval;
try {
 
    retval = wrapper->v->retrieve(*image->v, channel);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvVideoCaptureset(struct  VideoCapture_t* wrapper, int propId, double value)
{
    bool retval;
try {
 
    retval = wrapper->v->set(propId, value);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvVideoWriterisOpened(struct  VideoWriter_t* wrapper)
{
    bool retval;
try {
 
    retval = wrapper->v->isOpened();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvVideoWriteropen(struct  VideoWriter_t* wrapper, string_t* filename, int fourcc, double fps, Size_t* frameSize, bool isColor)
{
    bool retval;
try {
 
    retval = wrapper->v->open(filename->v, fourcc, fps, *frameSize->v, isColor);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvVideoWriterrelease(struct  VideoWriter_t* wrapper)
{
try {
 
    wrapper->v->release();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvVideoWriterwrite(struct  VideoWriter_t* wrapper, Mat_t* image)
{
try {
 
    wrapper->v->write(*image->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvSIFTcompute(struct  SIFT_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* descriptors)
{
try {
 
    wrapper->v->compute(*image->v, *keypoints, *descriptors->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvSIFTdescriptorSize(struct  SIFT_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->descriptorSize();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvSIFTdescriptorType(struct  SIFT_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->descriptorType();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvSIFTdetect(struct  SIFT_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* mask)
{
try {
 
    wrapper->v->detect(*image->v, *keypoints, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvSIFTdetectAndCompute(struct  SIFT_t* wrapper, Mat_t* image, Mat_t* mask, vector_KeyPoint* keypoints, Mat_t* descriptors, bool useProvidedKeypoints)
{
try {
 
    wrapper->v->operator ()(*image->v, *mask->v, *keypoints, *descriptors->v, useProvidedKeypoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}

CVAPI(bool) pCvSURFGet_extended(SURF_t* p)
{
    bool retval;
try {

    retval =   (p->v->extended);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSURFSet_extended(SURF_t* p, bool val)
{
try {
    p->v->extended = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(double) pCvSURFGet_hessianThreshold(SURF_t* p)
{
    double retval;
try {

    retval =   (p->v->hessianThreshold);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSURFSet_hessianThreshold(SURF_t* p, double val)
{
try {
    p->v->hessianThreshold = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvSURFGet_nOctaveLayers(SURF_t* p)
{
    int retval;
try {

    retval =   (p->v->nOctaveLayers);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSURFSet_nOctaveLayers(SURF_t* p, int val)
{
try {
    p->v->nOctaveLayers = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(int) pCvSURFGet_nOctaves(SURF_t* p)
{
    int retval;
try {

    retval =   (p->v->nOctaves);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSURFSet_nOctaves(SURF_t* p, int val)
{
try {
    p->v->nOctaves = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}

CVAPI(bool) pCvSURFGet_upright(SURF_t* p)
{
    bool retval;
try {

    retval =   (p->v->upright);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);
}

CVAPI(void) pCvSURFSet_upright(SURF_t* p, bool val)
{
try {
    p->v->upright = val;
} catch (std::exception &e) { 
         exceptionDisplay(e.what());
};
    return ;
}
CVAPI(void)   pCvSURFcompute(struct  SURF_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* descriptors)
{
try {
 
    wrapper->v->compute(*image->v, *keypoints, *descriptors->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvSURFdescriptorSize(struct  SURF_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->descriptorSize();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvSURFdescriptorType(struct  SURF_t* wrapper)
{
    int retval;
try {
 
    retval = wrapper->v->descriptorType();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvSURFdetect(struct  SURF_t* wrapper, Mat_t* image, vector_KeyPoint* keypoints, Mat_t* mask)
{
try {
 
    wrapper->v->detect(*image->v, *keypoints, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvSURFdetectAndCompute(struct  SURF_t* wrapper, Mat_t* image, Mat_t* mask, vector_KeyPoint* keypoints, Mat_t* descriptors, bool useProvidedKeypoints)
{
try {
 
    wrapper->v->operator ()(*image->v, *mask->v, *keypoints, *descriptors->v, useProvidedKeypoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvflann_Indexbuild(struct  flann_Index_t* wrapper, Mat_t* wholefeatures, Mat_t* additionalfeatures, IndexParams_t* params, cvflann_flann_distance_t distType)
{
try {
 
    wrapper->v->build(*wholefeatures->v, *additionalfeatures->v, *params->v, distType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvflann_IndexgetAlgorithm(struct  flann_Index_t* wrapper)
{
    cvflann_flann_algorithm_t retval;
try {
 
    retval = wrapper->v->getAlgorithm();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvflann_IndexgetDistance(struct  flann_Index_t* wrapper)
{
    cvflann_flann_distance_t retval;
try {
 
    retval = wrapper->v->getDistance();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvflann_IndexknnSearch(struct  flann_Index_t* wrapper, Mat_t* query, Mat_t* indices, Mat_t* dists, int knn, SearchParams_t* params)
{
try {
 
    wrapper->v->knnSearch(*query->v, *indices->v, *dists->v, knn, *params->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvflann_Indexload(struct  flann_Index_t* wrapper, Mat_t* features, string_t* filename)
{
    bool retval;
try {
 
    retval = wrapper->v->load(*features->v, filename->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvflann_IndexradiusSearch(struct  flann_Index_t* wrapper, Mat_t* query, Mat_t* indices, Mat_t* dists, double radius, int maxResults, SearchParams_t* params)
{
    int retval;
try {
 
    retval = wrapper->v->radiusSearch(*query->v, *indices->v, *dists->v, radius, maxResults, *params->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvflann_Indexrelease(struct  flann_Index_t* wrapper)
{
try {
 
    wrapper->v->release();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvflann_Indexsave(struct  flann_Index_t* wrapper, string_t* filename)
{
try {
 
    wrapper->v->save(filename->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
