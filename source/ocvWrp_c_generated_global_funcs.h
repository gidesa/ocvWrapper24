CVAPI(Ptr_Algorithm*)   pCvAlgorithm__create(string_t* name)
{
Ptr_Algorithm* retval = 0;
try {
 
    Ptr_Algorithm p = Algorithm::_create(name->v);
    retval = Ptr_cpy(p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvAlgorithm_getList(vector_string* algorithms)
{
try {
 
    Algorithm::getList(*algorithms);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(RotatedRect_t*)   pCvCamShift(Mat_t* probImage, Rect_t* window, TermCriteria_t* criteria)
{
    RotatedRect_t*  retval = new RotatedRect_t;
try { 
    RotatedRect tr = cv::CamShift(*probImage->v, *window->v, *criteria->v);
    RotatedRect*  t = new RotatedRect();
    retval->v = t;
    RotatedRect_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvCanny(Mat_t* image, Mat_t* edges, double threshold1, double threshold2, int apertureSize, bool L2gradient)
{
try {
 
    cv::Canny(*image->v, *edges->v, threshold1, threshold2, apertureSize, L2gradient);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Ptr_DescriptorExtractor*)   pCvDescriptorExtractor_create(string_t* descriptorExtractorType)
{
Ptr_DescriptorExtractor* retval = 0;
try {
 
    Ptr_DescriptorExtractor p = DescriptorExtractor::create(descriptorExtractorType->v);
    retval = Ptr_cpy(p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Ptr_DescriptorMatcher*)   pCvDescriptorMatcher_create(string_t* descriptorMatcherType)
{
Ptr_DescriptorMatcher* retval = 0;
try {
 
    Ptr_DescriptorMatcher p = DescriptorMatcher::create(descriptorMatcherType->v);
    retval = Ptr_cpy(p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Ptr_Feature2D*)   pCvFeature2D_create(string_t* name)
{
Ptr_Feature2D* retval = 0;
try {
 
    Ptr_Feature2D p = Feature2D::create(name->v);
    retval = Ptr_cpy(p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Ptr_FeatureDetector*)   pCvFeatureDetector_create(string_t* detectorType)
{
Ptr_FeatureDetector* retval = 0;
try {
 
    Ptr_FeatureDetector p = FeatureDetector::create(detectorType->v);
    retval = Ptr_cpy(p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvGaussianBlur(Mat_t* src, Mat_t* dst, Size_t* ksize, double sigmaX, double sigmaY, int borderType)
{
try {
 
    cv::GaussianBlur(*src->v, *dst->v, *ksize->v, sigmaX, sigmaY, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(vector_float*)   pCvHOGDescriptor_getDaimlerPeopleDetector()
{
vector_float* retval = new vector_float;
try {
 
    vector_float tr = HOGDescriptor::getDaimlerPeopleDetector();
    vector_float_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(vector_float*)   pCvHOGDescriptor_getDefaultPeopleDetector()
{
vector_float* retval = new vector_float;
try {
 
    vector_float tr = HOGDescriptor::getDefaultPeopleDetector();
    vector_float_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvHoughCircles(Mat_t* image, Mat_t* circles, int method, double dp, double minDist, double param1, double param2, int minRadius, int maxRadius)
{
try {
 
    cv::HoughCircles(*image->v, *circles->v, method, dp, minDist, param1, param2, minRadius, maxRadius);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvHoughLines(Mat_t* image, Mat_t* lines, double rho, double theta, int threshold, double srn, double stn)
{
try {
 
    cv::HoughLines(*image->v, *lines->v, rho, theta, threshold, srn, stn);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvHoughLinesP(Mat_t* image, Mat_t* lines, double rho, double theta, int threshold, double minLineLength, double maxLineGap)
{
try {
 
    cv::HoughLinesP(*image->v, *lines->v, rho, theta, threshold, minLineLength, maxLineGap);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvHuMoments(Moments_t* m, Mat_t* hu)
{
try {
 
    cv::HuMoments(*m->v, *hu->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvLUT(Mat_t* src, Mat_t* lut, Mat_t* dst, int interpolation)
{
try {
 
    cv::LUT(*src->v, *lut->v, *dst->v, interpolation);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvLaplacian(Mat_t* src, Mat_t* dst, int ddepth, int ksize, double scale, double delta, int borderType)
{
try {
 
    cv::Laplacian(*src->v, *dst->v, ddepth, ksize, scale, delta, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvMahalanobis(Mat_t* v1, Mat_t* v2, Mat_t* icovar)
{
    double retval;
try {
 
    retval = cv::Mahalanobis(*v1->v, *v2->v, *icovar->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvPCABackProject(Mat_t* data, Mat_t* mean, Mat_t* eigenvectors, Mat_t* result)
{
try {
 
    cv::PCABackProject(*data->v, *mean->v, *eigenvectors->v, *result->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvPCACompute(Mat_t* data, Mat_t* mean, Mat_t* eigenvectors, int maxComponents)
{
try {
 
    cv::PCACompute(*data->v, *mean->v, *eigenvectors->v, maxComponents);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvPCAComputeVar(Mat_t* data, Mat_t* mean, Mat_t* eigenvectors, double retainedVariance)
{
try {
 
    cv::PCAComputeVar(*data->v, *mean->v, *eigenvectors->v, retainedVariance);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvPCAProject(Mat_t* data, Mat_t* mean, Mat_t* eigenvectors, Mat_t* result)
{
try {
 
    cv::PCAProject(*data->v, *mean->v, *eigenvectors->v, *result->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvPSNR(Mat_t* src1, Mat_t* src2)
{
    double retval;
try {
 
    retval = cv::PSNR(*src1->v, *src2->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Vec3d_t*)   pCvRQDecomp3x3(Mat_t* src, Mat_t* mtxR, Mat_t* mtxQ, Mat_t* Qx, Mat_t* Qy, Mat_t* Qz)
{
    Vec3d_t*  retval = new Vec3d_t;
try { 
    Vec3d tr = cv::RQDecomp3x3(*src->v, *mtxR->v, *mtxQ->v, *Qx->v, *Qy->v, *Qz->v);
    Vec3d*  t = new Vec3d();
    retval->v = t;
    Vec3d_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvRodrigues(Mat_t* src, Mat_t* dst, Mat_t* jacobian)
{
try {
 
    cv::Rodrigues(*src->v, *dst->v, *jacobian->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvSVBackSubst(Mat_t* w, Mat_t* u, Mat_t* vt, Mat_t* rhs, Mat_t* dst)
{
try {
 
    cv::SVBackSubst(*w->v, *u->v, *vt->v, *rhs->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvSVDecomp(Mat_t* src, Mat_t* w, Mat_t* u, Mat_t* vt, int flags)
{
try {
 
    cv::SVDecomp(*src->v, *w->v, *u->v, *vt->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvScharr(Mat_t* src, Mat_t* dst, int ddepth, int dx, int dy, double scale, double delta, int borderType)
{
try {
 
    cv::Scharr(*src->v, *dst->v, ddepth, dx, dy, scale, delta, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvSobel(Mat_t* src, Mat_t* dst, int ddepth, int dx, int dy, int ksize, double scale, double delta, int borderType)
{
try {
 
    cv::Sobel(*src->v, *dst->v, ddepth, dx, dy, ksize, scale, delta, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvabsdiff(Mat_t* src1, Mat_t* src2, Mat_t* dst)
{
try {
 
    cv::absdiff(*src1->v, *src2->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvaccumulate(Mat_t* src, Mat_t* dst, Mat_t* mask)
{
try {
 
    cv::accumulate(*src->v, *dst->v, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvaccumulateProduct(Mat_t* src1, Mat_t* src2, Mat_t* dst, Mat_t* mask)
{
try {
 
    cv::accumulateProduct(*src1->v, *src2->v, *dst->v, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvaccumulateSquare(Mat_t* src, Mat_t* dst, Mat_t* mask)
{
try {
 
    cv::accumulateSquare(*src->v, *dst->v, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvaccumulateWeighted(Mat_t* src, Mat_t* dst, double alpha, Mat_t* mask)
{
try {
 
    cv::accumulateWeighted(*src->v, *dst->v, alpha, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvadaptiveBilateralFilter(Mat_t* src, Mat_t* dst, Size_t* ksize, double sigmaSpace, double maxSigmaColor, Point_t* anchor, int borderType)
{
try {
 
    cv::adaptiveBilateralFilter(*src->v, *dst->v, *ksize->v, sigmaSpace, maxSigmaColor, *anchor->v, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvadaptiveThreshold(Mat_t* src, Mat_t* dst, double maxValue, int adaptiveMethod, int thresholdType, int blockSize, double C)
{
try {
 
    cv::adaptiveThreshold(*src->v, *dst->v, maxValue, adaptiveMethod, thresholdType, blockSize, C);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvadd(Mat_t* src1, Mat_t* src2, Mat_t* dst, Mat_t* mask, int dtype)
{
try {
 
    cv::add(*src1->v, *src2->v, *dst->v, *mask->v, dtype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvaddWeighted(Mat_t* src1, double alpha, Mat_t* src2, double beta, double gamma, Mat_t* dst, int dtype)
{
try {
 
    cv::addWeighted(*src1->v, alpha, *src2->v, beta, gamma, *dst->v, dtype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvapplyColorMap(Mat_t* src, Mat_t* dst, int colormap)
{
try {
 
    cv::applyColorMap(*src->v, *dst->v, colormap);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvapproxPolyDP(Mat_t* curve, Mat_t* approxCurve, double epsilon, bool closed)
{
try {
 
    cv::approxPolyDP(*curve->v, *approxCurve->v, epsilon, closed);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvarcLength(Mat_t* curve, bool closed)
{
    double retval;
try {
 
    retval = cv::arcLength(*curve->v, closed);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvarrowedLine(Mat_t* img, Point_t* pt1, Point_t* pt2, Scalar_t* color, int thickness, int line_type, int shift, double tipLength)
{
try {
 
    cv::arrowedLine(*img->v, *pt1->v, *pt2->v, *color->v, thickness, line_type, shift, tipLength);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvbatchDistance(Mat_t* src1, Mat_t* src2, Mat_t* dist, int dtype, Mat_t* nidx, int normType, int K, Mat_t* mask, int update, bool crosscheck)
{
try {
 
    cv::batchDistance(*src1->v, *src2->v, *dist->v, dtype, *nidx->v, normType, K, *mask->v, update, crosscheck);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvbilateralFilter(Mat_t* src, Mat_t* dst, int d, double sigmaColor, double sigmaSpace, int borderType)
{
try {
 
    cv::bilateralFilter(*src->v, *dst->v, d, sigmaColor, sigmaSpace, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvbitwise_and(Mat_t* src1, Mat_t* src2, Mat_t* dst, Mat_t* mask)
{
try {
 
    cv::bitwise_and(*src1->v, *src2->v, *dst->v, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvbitwise_not(Mat_t* src, Mat_t* dst, Mat_t* mask)
{
try {
 
    cv::bitwise_not(*src->v, *dst->v, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvbitwise_or(Mat_t* src1, Mat_t* src2, Mat_t* dst, Mat_t* mask)
{
try {
 
    cv::bitwise_or(*src1->v, *src2->v, *dst->v, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvbitwise_xor(Mat_t* src1, Mat_t* src2, Mat_t* dst, Mat_t* mask)
{
try {
 
    cv::bitwise_xor(*src1->v, *src2->v, *dst->v, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvblur(Mat_t* src, Mat_t* dst, Size_t* ksize, Point_t* anchor, int borderType)
{
try {
 
    cv::blur(*src->v, *dst->v, *ksize->v, *anchor->v, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvborderInterpolate(int p, int len, int borderType)
{
    int retval;
try {
 
    retval = cv::borderInterpolate(p, len, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Rect_t*)   pCvboundingRect(Mat_t* points)
{
    Rect_t*  retval = new Rect_t;
try { 
    Rect tr = cv::boundingRect(*points->v);
    Rect*  t = new Rect();
    retval->v = t;
    Rect_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvboxFilter(Mat_t* src, Mat_t* dst, int ddepth, Size_t* ksize, Point_t* anchor, bool normalize, int borderType)
{
try {
 
    cv::boxFilter(*src->v, *dst->v, ddepth, *ksize->v, *anchor->v, normalize, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvbuildOpticalFlowPyramid(Mat_t* img, vector_Mat* pyramid, Size_t* winSize, int maxLevel, bool withDerivatives, int pyrBorder, int derivBorder, bool tryReuseInputImage)
{
    int retval;
try {
 
    retval = cv::buildOpticalFlowPyramid(*img->v, *pyramid, *winSize->v, maxLevel, withDerivatives, pyrBorder, derivBorder, tryReuseInputImage);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvcalcBackProject(vector_Mat* images, vector_int* channels, Mat_t* hist, Mat_t* dst, vector_float* ranges, double scale)
{
try {
 
    cv::calcBackProject(*images, *channels, *hist->v, *dst->v, *ranges, scale);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcalcCovarMatrix(Mat_t* samples, Mat_t* covar, Mat_t* mean, int flags, int ctype)
{
try {
 
    cv::calcCovarMatrix(*samples->v, *covar->v, *mean->v, flags, ctype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvcalcGlobalOrientation(Mat_t* orientation, Mat_t* mask, Mat_t* mhi, double timestamp, double duration)
{
    double retval;
try {
 
    retval = cv::calcGlobalOrientation(*orientation->v, *mask->v, *mhi->v, timestamp, duration);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvcalcHist(vector_Mat* images, vector_int* channels, Mat_t* mask, Mat_t* hist, vector_int* histSize, vector_float* ranges, bool accumulate)
{
try {
 
    cv::calcHist(*images, *channels, *mask->v, *hist->v, *histSize, *ranges, accumulate);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcalcMotionGradient(Mat_t* mhi, Mat_t* mask, Mat_t* orientation, double delta1, double delta2, int apertureSize)
{
try {
 
    cv::calcMotionGradient(*mhi->v, *mask->v, *orientation->v, delta1, delta2, apertureSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcalcOpticalFlowFarneback(Mat_t* prev, Mat_t* next, Mat_t* flow, double pyr_scale, int levels, int winsize, int iterations, int poly_n, double poly_sigma, int flags)
{
try {
 
    cv::calcOpticalFlowFarneback(*prev->v, *next->v, *flow->v, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcalcOpticalFlowPyrLK(Mat_t* prevImg, Mat_t* nextImg, Mat_t* prevPts, Mat_t* nextPts, Mat_t* status, Mat_t* err, Size_t* winSize, int maxLevel, TermCriteria_t* criteria, int flags, double minEigThreshold)
{
try {
 
    cv::calcOpticalFlowPyrLK(*prevImg->v, *nextImg->v, *prevPts->v, *nextPts->v, *status->v, *err->v, *winSize->v, maxLevel, *criteria->v, flags, minEigThreshold);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcalcOpticalFlowSF(Mat_t* from, Mat_t* _to, Mat_t* flow, int layers, int averaging_block_size, int max_flow)
{
try {
 
    cv::calcOpticalFlowSF(*from->v, *_to->v, *flow->v, layers, averaging_block_size, max_flow);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcalcOpticalFlowSF2(Mat_t* from, Mat_t* _to, Mat_t* flow, int layers, int averaging_block_size, int max_flow, double sigma_dist, double sigma_color, int postprocess_window, double sigma_dist_fix, double sigma_color_fix, double occ_thr, int upscale_averaging_radius, double upscale_sigma_dist, double upscale_sigma_color, double speed_up_thr)
{
try {
 
    cv::calcOpticalFlowSF(*from->v, *_to->v, *flow->v, layers, averaging_block_size, max_flow, sigma_dist, sigma_color, postprocess_window, sigma_dist_fix, sigma_color_fix, occ_thr, upscale_averaging_radius, upscale_sigma_dist, upscale_sigma_color, speed_up_thr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvcalibrateCamera(vector_Mat* objectPoints, vector_Mat* imagePoints, Size_t* imageSize, Mat_t* cameraMatrix, Mat_t* distCoeffs, vector_Mat* rvecs, vector_Mat* tvecs, int flags, TermCriteria_t* criteria)
{
    double retval;
try {
 
    retval = cv::calibrateCamera(*objectPoints, *imagePoints, *imageSize->v, *cameraMatrix->v, *distCoeffs->v, *rvecs, *tvecs, flags, *criteria->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvcalibrationMatrixValues(Mat_t* cameraMatrix, Size_t* imageSize, double apertureWidth, double apertureHeight, double fovx, double fovy, double focalLength, Point2d_t* principalPoint, double aspectRatio)
{
try {
 
    cv::calibrationMatrixValues(*cameraMatrix->v, *imageSize->v, apertureWidth, apertureHeight, fovx, fovy, focalLength, *principalPoint->v, aspectRatio);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcartToPolar(Mat_t* x, Mat_t* y, Mat_t* magnitude, Mat_t* angle, bool angleInDegrees)
{
try {
 
    cv::cartToPolar(*x->v, *y->v, *magnitude->v, *angle->v, angleInDegrees);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvchamerMatching(Mat_t* img, Mat_t* templ, vector_vector_Point* results, vector_float* cost, double templScale, int maxMatches, double minMatchDistance, int padX, int padY, int scales, double minScale, double maxScale, double orientationWeight, double truncate)
{
    int retval;
try {
 
    retval = cv::chamerMatching(*img->v, *templ->v, *results, *cost, templScale, maxMatches, minMatchDistance, padX, padY, scales, minScale, maxScale, orientationWeight, truncate);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvcheckHardwareSupport(int feature)
{
    bool retval;
try {
 
    retval = cv::checkHardwareSupport(feature);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvcheckRange(Mat_t* a, bool quiet, Point_t* pos, double minVal, double maxVal)
{
    bool retval;
try {
 
    retval = cv::checkRange(*a->v, quiet, pos->v, minVal, maxVal);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvcircle(Mat_t* img, Point_t* center, int radius, Scalar_t* color, int thickness, int lineType, int shift)
{
try {
 
    cv::circle(*img->v, *center->v, radius, *color->v, thickness, lineType, shift);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvclipLine(Rect_t* imgRect, Point_t* pt1, Point_t* pt2)
{
    bool retval;
try {
 
    retval = cv::clipLine(*imgRect->v, *pt1->v, *pt2->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvcompare(Mat_t* src1, Mat_t* src2, Mat_t* dst, int cmpop)
{
try {
 
    cv::compare(*src1->v, *src2->v, *dst->v, cmpop);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvcompareHist(Mat_t* H1, Mat_t* H2, int method)
{
    double retval;
try {
 
    retval = cv::compareHist(*H1->v, *H2->v, method);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvcompleteSymm(Mat_t* mtx, bool lowerToUpper)
{
try {
 
    cv::completeSymm(*mtx->v, lowerToUpper);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcomposeRT(Mat_t* rvec1, Mat_t* tvec1, Mat_t* rvec2, Mat_t* tvec2, Mat_t* rvec3, Mat_t* tvec3, Mat_t* dr3dr1, Mat_t* dr3dt1, Mat_t* dr3dr2, Mat_t* dr3dt2, Mat_t* dt3dr1, Mat_t* dt3dt1, Mat_t* dt3dr2, Mat_t* dt3dt2)
{
try {
 
    cv::composeRT(*rvec1->v, *tvec1->v, *rvec2->v, *tvec2->v, *rvec3->v, *tvec3->v, *dr3dr1->v, *dr3dt1->v, *dr3dr2->v, *dr3dt2->v, *dt3dr1->v, *dt3dt1->v, *dt3dr2->v, *dt3dt2->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcomputeCorrespondEpilines(Mat_t* points, int whichImage, Mat_t* F, Mat_t* lines)
{
try {
 
    cv::computeCorrespondEpilines(*points->v, whichImage, *F->v, *lines->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvcontourArea(Mat_t* contour, bool oriented)
{
    double retval;
try {
 
    retval = cv::contourArea(*contour->v, oriented);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvconvertMaps(Mat_t* map1, Mat_t* map2, Mat_t* dstmap1, Mat_t* dstmap2, int dstmap1type, bool nninterpolation)
{
try {
 
    cv::convertMaps(*map1->v, *map2->v, *dstmap1->v, *dstmap2->v, dstmap1type, nninterpolation);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvconvertPointsFromHomogeneous(Mat_t* src, Mat_t* dst)
{
try {
 
    cv::convertPointsFromHomogeneous(*src->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvconvertPointsToHomogeneous(Mat_t* src, Mat_t* dst)
{
try {
 
    cv::convertPointsToHomogeneous(*src->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvconvertScaleAbs(Mat_t* src, Mat_t* dst, double alpha, double beta)
{
try {
 
    cv::convertScaleAbs(*src->v, *dst->v, alpha, beta);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvconvexHull(Mat_t* points, Mat_t* hull, bool clockwise, bool returnPoints)
{
try {
 
    cv::convexHull(*points->v, *hull->v, clockwise, returnPoints);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvconvexityDefects(Mat_t* contour, Mat_t* convexhull, Mat_t* convexityDefects)
{
try {
 
    cv::convexityDefects(*contour->v, *convexhull->v, *convexityDefects->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcopyMakeBorder(Mat_t* src, Mat_t* dst, int top, int bottom, int left, int right, int borderType, Scalar_t* value)
{
try {
 
    cv::copyMakeBorder(*src->v, *dst->v, top, bottom, left, right, borderType, *value->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcornerEigenValsAndVecs(Mat_t* src, Mat_t* dst, int blockSize, int ksize, int borderType)
{
try {
 
    cv::cornerEigenValsAndVecs(*src->v, *dst->v, blockSize, ksize, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcornerHarris(Mat_t* src, Mat_t* dst, int blockSize, int ksize, double k, int borderType)
{
try {
 
    cv::cornerHarris(*src->v, *dst->v, blockSize, ksize, k, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcornerMinEigenVal(Mat_t* src, Mat_t* dst, int blockSize, int ksize, int borderType)
{
try {
 
    cv::cornerMinEigenVal(*src->v, *dst->v, blockSize, ksize, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcornerSubPix(Mat_t* image, Mat_t* corners, Size_t* winSize, Size_t* zeroZone, TermCriteria_t* criteria)
{
try {
 
    cv::cornerSubPix(*image->v, *corners->v, *winSize->v, *zeroZone->v, *criteria->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvcorrectMatches(Mat_t* F, Mat_t* points1, Mat_t* points2, Mat_t* newPoints1, Mat_t* newPoints2)
{
try {
 
    cv::correctMatches(*F->v, *points1->v, *points2->v, *newPoints1->v, *newPoints2->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvcountNonZero(Mat_t* src)
{
    int retval;
try {
 
    retval = cv::countNonZero(*src->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Ptr_CLAHE*)   pCvcreateCLAHE(double clipLimit, Size_t* tileGridSize)
{
Ptr_CLAHE* retval = 0;
try {
 
    Ptr_CLAHE p = cv::createCLAHE(clipLimit, *tileGridSize->v);
    retval = Ptr_cpy(p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Ptr_FaceRecognizer*)   pCvcreateEigenFaceRecognizer(int num_components, double threshold)
{
Ptr_FaceRecognizer* retval = 0;
try {
 
    Ptr_FaceRecognizer p = cv::createEigenFaceRecognizer(num_components, threshold);
    retval = Ptr_cpy(p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Ptr_FaceRecognizer*)   pCvcreateFisherFaceRecognizer(int num_components, double threshold)
{
Ptr_FaceRecognizer* retval = 0;
try {
 
    Ptr_FaceRecognizer p = cv::createFisherFaceRecognizer(num_components, threshold);
    retval = Ptr_cpy(p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvcreateHanningWindow(Mat_t* dst, Size_t* winSize, int _type)
{
try {
 
    cv::createHanningWindow(*dst->v, *winSize->v, _type);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Ptr_FaceRecognizer*)   pCvcreateLBPHFaceRecognizer(int radius, int neighbors, int grid_x, int grid_y, double threshold)
{
Ptr_FaceRecognizer* retval = 0;
try {
 
    Ptr_FaceRecognizer p = cv::createLBPHFaceRecognizer(radius, neighbors, grid_x, grid_y, threshold);
    retval = Ptr_cpy(p);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(float)   pCvcubeRoot(float val)
{
    float retval;
try {
 
    retval = cv::cubeRoot(val);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvcvtColor(Mat_t* src, Mat_t* dst, int code, int dstCn)
{
try {
 
    cv::cvtColor(*src->v, *dst->v, code, dstCn);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdct(Mat_t* src, Mat_t* dst, int flags)
{
try {
 
    cv::dct(*src->v, *dst->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdecomposeProjectionMatrix(Mat_t* projMatrix, Mat_t* cameraMatrix, Mat_t* rotMatrix, Mat_t* transVect, Mat_t* rotMatrixX, Mat_t* rotMatrixY, Mat_t* rotMatrixZ, Mat_t* eulerAngles)
{
try {
 
    cv::decomposeProjectionMatrix(*projMatrix->v, *cameraMatrix->v, *rotMatrix->v, *transVect->v, *rotMatrixX->v, *rotMatrixY->v, *rotMatrixZ->v, *eulerAngles->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdestroyAllWindows()
{
try {
 
    cv::destroyAllWindows();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdestroyWindow(string_t* winname)
{
try {
 
    cv::destroyWindow(winname->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvdeterminant(Mat_t* mtx)
{
    double retval;
try {
 
    retval = cv::determinant(*mtx->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvdft(Mat_t* src, Mat_t* dst, int flags, int nonzeroRows)
{
try {
 
    cv::dft(*src->v, *dst->v, flags, nonzeroRows);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdilate(Mat_t* src, Mat_t* dst, Mat_t* kernel, Point_t* anchor, int iterations, int borderType, Scalar_t* borderValue)
{
try {
 
    cv::dilate(*src->v, *dst->v, *kernel->v, *anchor->v, iterations, borderType, *borderValue->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdistanceTransform(Mat_t* src, Mat_t* dst, int distanceType, int maskSize)
{
try {
 
    cv::distanceTransform(*src->v, *dst->v, distanceType, maskSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdistanceTransformWithLabels(Mat_t* src, Mat_t* dst, Mat_t* labels, int distanceType, int maskSize, int labelType)
{
try {
 
    cv::distanceTransform(*src->v, *dst->v, *labels->v, distanceType, maskSize, labelType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdivide(Mat_t* src1, Mat_t* src2, Mat_t* dst, double scale, int dtype)
{
try {
 
    cv::divide(*src1->v, *src2->v, *dst->v, scale, dtype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdivide2(double scale, Mat_t* src2, Mat_t* dst, int dtype)
{
try {
 
    cv::divide(scale, *src2->v, *dst->v, dtype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdrawChessboardCorners(Mat_t* image, Size_t* patternSize, Mat_t* corners, bool patternWasFound)
{
try {
 
    cv::drawChessboardCorners(*image->v, *patternSize->v, *corners->v, patternWasFound);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdrawContours(Mat_t* image, vector_Mat* contours, int contourIdx, Scalar_t* color, int thickness, int lineType, Mat_t* hierarchy, int maxLevel, Point_t* offset)
{
try {
 
    cv::drawContours(*image->v, *contours, contourIdx, *color->v, thickness, lineType, *hierarchy->v, maxLevel, *offset->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdrawDataMatrixCodes(Mat_t* image, vector_string* codes, Mat_t* corners)
{
try {
 
    cv::drawDataMatrixCodes(*image->v, *codes, *corners->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdrawKeypoints(Mat_t* image, vector_KeyPoint* keypoints, Mat_t* outImage, Scalar_t* color, int flags)
{
try {
 
    cv::drawKeypoints(*image->v, *keypoints, *outImage->v, *color->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvdrawMarker(Mat_t* img, Point_t* position, Scalar_t* color, int markerType, int markerSize, int thickness, int line_type)
{
try {
 
    cv::drawMarker(*img->v, *position->v, *color->v, markerType, markerSize, thickness, line_type);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCveigen(Mat_t* src, bool computeEigenvectors, Mat_t* eigenvalues, Mat_t* eigenvectors)
{
    bool retval;
try {
 
    retval = cv::eigen(*src->v, computeEigenvectors, *eigenvalues->v, *eigenvectors->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvellipse(Mat_t* img, Point_t* center, Size_t* axes, double angle, double startAngle, double endAngle, Scalar_t* color, int thickness, int lineType, int shift)
{
try {
 
    cv::ellipse(*img->v, *center->v, *axes->v, angle, startAngle, endAngle, *color->v, thickness, lineType, shift);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvellipse2(Mat_t* img, RotatedRect_t* box, Scalar_t* color, int thickness, int lineType)
{
try {
 
    cv::ellipse(*img->v, *box->v, *color->v, thickness, lineType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvellipse2Poly(Point_t* center, Size_t* axes, int angle, int arcStart, int arcEnd, int delta, vector_Point* pts)
{
try {
 
    cv::ellipse2Poly(*center->v, *axes->v, angle, arcStart, arcEnd, delta, *pts);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvequalizeHist(Mat_t* src, Mat_t* dst)
{
try {
 
    cv::equalizeHist(*src->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCverode(Mat_t* src, Mat_t* dst, Mat_t* kernel, Point_t* anchor, int iterations, int borderType, Scalar_t* borderValue)
{
try {
 
    cv::erode(*src->v, *dst->v, *kernel->v, *anchor->v, iterations, borderType, *borderValue->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvestimateAffine3D(Mat_t* src, Mat_t* dst, Mat_t* out, Mat_t* inliers, double ransacThreshold, double confidence)
{
    int retval;
try {
 
    retval = cv::estimateAffine3D(*src->v, *dst->v, *out->v, *inliers->v, ransacThreshold, confidence);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvestimateRigidTransform(Mat_t* src, Mat_t* dst, bool fullAffine)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::estimateRigidTransform(*src->v, *dst->v, fullAffine);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvexp(Mat_t* src, Mat_t* dst)
{
try {
 
    cv::exp(*src->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvextractChannel(Mat_t* src, Mat_t* dst, int coi)
{
try {
 
    cv::extractChannel(*src->v, *dst->v, coi);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(float)   pCvfastAtan2(float y, float x)
{
    float retval;
try {
 
    retval = cv::fastAtan2(y, x);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvfastNlMeansDenoising(Mat_t* src, Mat_t* dst, float h, int templateWindowSize, int searchWindowSize)
{
try {
 
    cv::fastNlMeansDenoising(*src->v, *dst->v, h, templateWindowSize, searchWindowSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvfastNlMeansDenoisingColored(Mat_t* src, Mat_t* dst, float h, float hColor, int templateWindowSize, int searchWindowSize)
{
try {
 
    cv::fastNlMeansDenoisingColored(*src->v, *dst->v, h, hColor, templateWindowSize, searchWindowSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvfastNlMeansDenoisingColoredMulti(vector_Mat* srcImgs, Mat_t* dst, int imgToDenoiseIndex, int temporalWindowSize, float h, float hColor, int templateWindowSize, int searchWindowSize)
{
try {
 
    cv::fastNlMeansDenoisingColoredMulti(*srcImgs, *dst->v, imgToDenoiseIndex, temporalWindowSize, h, hColor, templateWindowSize, searchWindowSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvfastNlMeansDenoisingMulti(vector_Mat* srcImgs, Mat_t* dst, int imgToDenoiseIndex, int temporalWindowSize, float h, int templateWindowSize, int searchWindowSize)
{
try {
 
    cv::fastNlMeansDenoisingMulti(*srcImgs, *dst->v, imgToDenoiseIndex, temporalWindowSize, h, templateWindowSize, searchWindowSize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvfillConvexPoly(Mat_t* img, Mat_t* points, Scalar_t* color, int lineType, int shift)
{
try {
 
    cv::fillConvexPoly(*img->v, *points->v, *color->v, lineType, shift);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvfillPoly(Mat_t* img, vector_Mat* pts, Scalar_t* color, int lineType, int shift, Point_t* offset)
{
try {
 
    cv::fillPoly(*img->v, *pts, *color->v, lineType, shift, *offset->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvfilter2D(Mat_t* src, Mat_t* dst, int ddepth, Mat_t* kernel, Point_t* anchor, double delta, int borderType)
{
try {
 
    cv::filter2D(*src->v, *dst->v, ddepth, *kernel->v, *anchor->v, delta, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvfilterSpeckles(Mat_t* img, double newVal, int maxSpeckleSize, double maxDiff, Mat_t* buf)
{
try {
 
    cv::filterSpeckles(*img->v, newVal, maxSpeckleSize, maxDiff, *buf->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvfindChessboardCorners(Mat_t* image, Size_t* patternSize, Mat_t* corners, int flags)
{
    bool retval;
try {
 
    retval = cv::findChessboardCorners(*image->v, *patternSize->v, *corners->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvfindCirclesGrid(Mat_t* image, Size_t* patternSize, Mat_t* centers, int flags, Ptr_FeatureDetector* blobDetector)
{
    bool retval;
try {
 
    retval = cv::findCirclesGrid(*image->v, *patternSize->v, *centers->v, flags, *blobDetector);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvfindCirclesGridDefault(Mat_t* image, Size_t* patternSize, Mat_t* centers, int flags)
{
    bool retval;
try {
 
    retval = cv::findCirclesGridDefault(*image->v, *patternSize->v, *centers->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvfindContours(Mat_t* image, vector_Mat* contours, Mat_t* hierarchy, int mode, int method, Point_t* offset)
{
try {
 
    cv::findContours(*image->v, *contours, *hierarchy->v, mode, method, *offset->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvfindDataMatrix(Mat_t* image, vector_string* codes, Mat_t* corners, vector_Mat* dmtx)
{
try {
 
    cv::findDataMatrix(*image->v, *codes, *corners->v, *dmtx);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvfindFundamentalMat(Mat_t* points1, Mat_t* points2, int method, double param1, double param2, Mat_t* mask)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::findFundamentalMat(*points1->v, *points2->v, method, param1, param2, *mask->v);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvfindHomography(Mat_t* srcPoints, Mat_t* dstPoints, int method, double ransacReprojThreshold, Mat_t* mask)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::findHomography(*srcPoints->v, *dstPoints->v, method, ransacReprojThreshold, *mask->v);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvfindNonZero(Mat_t* src, Mat_t* idx)
{
try {
 
    cv::findNonZero(*src->v, *idx->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(RotatedRect_t*)   pCvfitEllipse(Mat_t* points)
{
    RotatedRect_t*  retval = new RotatedRect_t;
try { 
    RotatedRect tr = cv::fitEllipse(*points->v);
    RotatedRect*  t = new RotatedRect();
    retval->v = t;
    RotatedRect_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvfitLine(Mat_t* points, Mat_t* line, int distType, double param, double reps, double aeps)
{
try {
 
    cv::fitLine(*points->v, *line->v, distType, param, reps, aeps);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvflip(Mat_t* src, Mat_t* dst, int flipCode)
{
try {
 
    cv::flip(*src->v, *dst->v, flipCode);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvfloodFill(Mat_t* image, Mat_t* mask, Point_t* seedPoint, Scalar_t* newVal, Rect_t* rect, Scalar_t* loDiff, Scalar_t* upDiff, int flags)
{
    int retval;
try {
 
    retval = cv::floodFill(*image->v, *mask->v, *seedPoint->v, *newVal->v, rect->v, *loDiff->v, *upDiff->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvgemm(Mat_t* src1, Mat_t* src2, double alpha, Mat_t* src3, double beta, Mat_t* dst, int flags)
{
try {
 
    cv::gemm(*src1->v, *src2->v, alpha, *src3->v, beta, *dst->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvgetAffineTransform(Mat_t* src, Mat_t* dst)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::getAffineTransform(*src->v, *dst->v);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(string_t*)   pCvgetBuildInformation()
{
    string_t* retval = new string_t;
 try {
 
    const string  sr = cv::getBuildInformation();
    int len = sr.length() + 1; 
    retval->v = (char*) cvAlloc(len * sizeof(char)); 
    retval->nrchar = len; 
    strcpy(retval->v, &sr[0]); 
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int64)   pCvgetCPUTickCount()
{
    int64 retval;
try {
 
    retval = cv::getCPUTickCount();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvgetDefaultNewCameraMatrix(Mat_t* cameraMatrix, Size_t* imgsize, bool centerPrincipalPoint)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::getDefaultNewCameraMatrix(*cameraMatrix->v, *imgsize->v, centerPrincipalPoint);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvgetDerivKernels(Mat_t* kx, Mat_t* ky, int dx, int dy, int ksize, bool normalize, int ktype)
{
try {
 
    cv::getDerivKernels(*kx->v, *ky->v, dx, dy, ksize, normalize, ktype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvgetGaborKernel(Size_t* ksize, double sigma, double theta, double lambd, double gamma, double psi, int ktype)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::getGaborKernel(*ksize->v, sigma, theta, lambd, gamma, psi, ktype);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvgetGaussianKernel(int ksize, double sigma, int ktype)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::getGaussianKernel(ksize, sigma, ktype);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvgetNumThreads()
{
    int retval;
try {
 
    retval = cv::getNumThreads();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvgetNumberOfCPUs()
{
    int retval;
try {
 
    retval = cv::getNumberOfCPUs();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvgetOptimalDFTSize(int vecsize)
{
    int retval;
try {
 
    retval = cv::getOptimalDFTSize(vecsize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvgetOptimalNewCameraMatrix(Mat_t* cameraMatrix, Mat_t* distCoeffs, Size_t* imageSize, double alpha, Size_t* newImgSize, Rect_t* validPixROI, bool centerPrincipalPoint)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::getOptimalNewCameraMatrix(*cameraMatrix->v, *distCoeffs->v, *imageSize->v, alpha, *newImgSize->v, validPixROI->v, centerPrincipalPoint);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvgetPerspectiveTransform(Mat_t* src, Mat_t* dst)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::getPerspectiveTransform(*src->v, *dst->v);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvgetRectSubPix(Mat_t* image, Size_t* patchSize, Point2f_t* center, Mat_t* patch, int patchType)
{
try {
 
    cv::getRectSubPix(*image->v, *patchSize->v, *center->v, *patch->v, patchType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvgetRotationMatrix2D(Point2f_t* center, double angle, double scale)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::getRotationMatrix2D(*center->v, angle, scale);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvgetStructuringElement(int shape, Size_t* ksize, Point_t* anchor)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::getStructuringElement(shape, *ksize->v, *anchor->v);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Size_t*)   pCvgetTextSize(string_t* text, int fontFace, double fontScale, int thickness, int* baseLine)
{
    Size_t*  retval = new Size_t;
try { 
    Size tr = cv::getTextSize(text->v, fontFace, fontScale, thickness, baseLine);
    Size*  t = new Size();
    retval->v = t;
    Size_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvgetThreadNum()
{
    int retval;
try {
 
    retval = cv::getThreadNum();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int64)   pCvgetTickCount()
{
    int64 retval;
try {
 
    retval = cv::getTickCount();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(double)   pCvgetTickFrequency()
{
    double retval;
try {
 
    retval = cv::getTickFrequency();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvgetTrackbarPos(string_t* trackbarname, string_t* winname)
{
    int retval;
try {
 
    retval = cv::getTrackbarPos(trackbarname->v, winname->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Rect_t*)   pCvgetValidDisparityROI(Rect_t* roi1, Rect_t* roi2, int minDisparity, int numberOfDisparities, int SADWindowSize)
{
    Rect_t*  retval = new Rect_t;
try { 
    Rect tr = cv::getValidDisparityROI(*roi1->v, *roi2->v, minDisparity, numberOfDisparities, SADWindowSize);
    Rect*  t = new Rect();
    retval->v = t;
    Rect_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(double)   pCvgetWindowProperty(string_t* winname, int prop_id)
{
    double retval;
try {
 
    retval = cv::getWindowProperty(winname->v, prop_id);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvgoodFeaturesToTrack(Mat_t* image, Mat_t* corners, int maxCorners, double qualityLevel, double minDistance, Mat_t* mask, int blockSize, bool useHarrisDetector, double k)
{
try {
 
    cv::goodFeaturesToTrack(*image->v, *corners->v, maxCorners, qualityLevel, minDistance, *mask->v, blockSize, useHarrisDetector, k);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvgrabCut(Mat_t* img, Mat_t* mask, Rect_t* rect, Mat_t* bgdModel, Mat_t* fgdModel, int iterCount, int mode)
{
try {
 
    cv::grabCut(*img->v, *mask->v, *rect->v, *bgdModel->v, *fgdModel->v, iterCount, mode);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvgroupRectangles(vector_Rect* rectList, vector_int* weights, int groupThreshold, double eps)
{
try {
 
    cv::groupRectangles(*rectList, *weights, groupThreshold, eps);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvhconcat(vector_Mat* src, Mat_t* dst)
{
try {
 
    cv::hconcat(*src, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvidct(Mat_t* src, Mat_t* dst, int flags)
{
try {
 
    cv::idct(*src->v, *dst->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvidft(Mat_t* src, Mat_t* dst, int flags, int nonzeroRows)
{
try {
 
    cv::idft(*src->v, *dst->v, flags, nonzeroRows);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvimdecode(Mat_t* buf, int flags)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::imdecode(*buf->v, flags);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvimencode(string_t* ext, Mat_t* img, vector_uchar* buf, vector_int* params)
{
    bool retval;
try {
 
    retval = cv::imencode(ext->v, *img->v, *buf, *params);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Mat_t*)   pCvimread(string_t* filename, int flags)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::imread(filename->v, flags);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvimshow(string_t* winname, Mat_t* mat)
{
try {
 
    cv::imshow(winname->v, *mat->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvimwrite(string_t* filename, Mat_t* img, vector_int* params)
{
    bool retval;
try {
 
    retval = cv::imwrite(filename->v, *img->v, *params);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvinRange(Mat_t* src, Mat_t* lowerb, Mat_t* upperb, Mat_t* dst)
{
try {
 
    cv::inRange(*src->v, *lowerb->v, *upperb->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Mat_t*)   pCvinitCameraMatrix2D(vector_Mat* objectPoints, vector_Mat* imagePoints, Size_t* imageSize, double aspectRatio)
{
    Mat_t*  retval = new Mat_t;
try { 
    Mat tr = cv::initCameraMatrix2D(*objectPoints, *imagePoints, *imageSize->v, aspectRatio);
    Mat*  t = new Mat();
    retval->v = t;
    Mat_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvinitModule_nonfree()
{
    bool retval;
try {
 
    retval = cv::initModule_nonfree();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvinitUndistortRectifyMap(Mat_t* cameraMatrix, Mat_t* distCoeffs, Mat_t* R, Mat_t* newCameraMatrix, Size_t* size, int m1type, Mat_t* map1, Mat_t* map2)
{
try {
 
    cv::initUndistortRectifyMap(*cameraMatrix->v, *distCoeffs->v, *R->v, *newCameraMatrix->v, *size->v, m1type, *map1->v, *map2->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(float)   pCvinitWideAngleProjMap(Mat_t* cameraMatrix, Mat_t* distCoeffs, Size_t* imageSize, int destImageWidth, int m1type, Mat_t* map1, Mat_t* map2, int projType, double alpha)
{
    float retval;
try {
 
    retval = cv::initWideAngleProjMap(*cameraMatrix->v, *distCoeffs->v, *imageSize->v, destImageWidth, m1type, *map1->v, *map2->v, projType, alpha);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvinpaint(Mat_t* src, Mat_t* inpaintMask, Mat_t* dst, double inpaintRadius, int flags)
{
try {
 
    cv::inpaint(*src->v, *inpaintMask->v, *dst->v, inpaintRadius, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvinsertChannel(Mat_t* src, Mat_t* dst, int coi)
{
try {
 
    cv::insertChannel(*src->v, *dst->v, coi);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvintegral(Mat_t* src, Mat_t* sum, int sdepth)
{
try {
 
    cv::integral(*src->v, *sum->v, sdepth);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvintegral2(Mat_t* src, Mat_t* sum, Mat_t* sqsum, int sdepth)
{
try {
 
    cv::integral(*src->v, *sum->v, *sqsum->v, sdepth);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvintegral3(Mat_t* src, Mat_t* sum, Mat_t* sqsum, Mat_t* tilted, int sdepth)
{
try {
 
    cv::integral(*src->v, *sum->v, *sqsum->v, *tilted->v, sdepth);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(float)   pCvintersectConvexConvex(Mat_t* _p1, Mat_t* _p2, Mat_t* _p12, bool handleNested)
{
    float retval;
try {
 
    retval = cv::intersectConvexConvex(*_p1->v, *_p2->v, *_p12->v, handleNested);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(double)   pCvinvert(Mat_t* src, Mat_t* dst, int flags)
{
    double retval;
try {
 
    retval = cv::invert(*src->v, *dst->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvinvertAffineTransform(Mat_t* M, Mat_t* iM)
{
try {
 
    cv::invertAffineTransform(*M->v, *iM->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvisContourConvex(Mat_t* contour)
{
    bool retval;
try {
 
    retval = cv::isContourConvex(*contour->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(double)   pCvkmeans(Mat_t* data, int K, Mat_t* bestLabels, TermCriteria_t* criteria, int attempts, int flags, Mat_t* centers)
{
    double retval;
try {
 
    retval = cv::kmeans(*data->v, K, *bestLabels->v, *criteria->v, attempts, flags, *centers->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvline(Mat_t* img, Point_t* pt1, Point_t* pt2, Scalar_t* color, int thickness, int lineType, int shift)
{
try {
 
    cv::line(*img->v, *pt1->v, *pt2->v, *color->v, thickness, lineType, shift);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvlog(Mat_t* src, Mat_t* dst)
{
try {
 
    cv::log(*src->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmagnitude(Mat_t* x, Mat_t* y, Mat_t* magnitude)
{
try {
 
    cv::magnitude(*x->v, *y->v, *magnitude->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmatMulDeriv(Mat_t* A, Mat_t* B, Mat_t* dABdA, Mat_t* dABdB)
{
try {
 
    cv::matMulDeriv(*A->v, *B->v, *dABdA->v, *dABdB->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvmatchShapes(Mat_t* contour1, Mat_t* contour2, int method, double parameter)
{
    double retval;
try {
 
    retval = cv::matchShapes(*contour1->v, *contour2->v, method, parameter);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvmatchTemplate(Mat_t* image, Mat_t* templ, Mat_t* result, int method)
{
try {
 
    cv::matchTemplate(*image->v, *templ->v, *result->v, method);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmax(Mat_t* src1, Mat_t* src2, Mat_t* dst)
{
try {
 
    cv::max(*src1->v, *src2->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Scalar_t*)   pCvmean(Mat_t* src, Mat_t* mask)
{
    Scalar_t*  retval = new Scalar_t;
try { 
    Scalar tr = cv::mean(*src->v, *mask->v);
    Scalar*  t = new Scalar();
    retval->v = t;
    Scalar_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvmeanShift(Mat_t* probImage, Rect_t* window, TermCriteria_t* criteria)
{
    int retval;
try {
 
    retval = cv::meanShift(*probImage->v, *window->v, *criteria->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvmeanStdDev(Mat_t* src, Mat_t* mean, Mat_t* stddev, Mat_t* mask)
{
try {
 
    cv::meanStdDev(*src->v, *mean->v, *stddev->v, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmedianBlur(Mat_t* src, Mat_t* dst, int ksize)
{
try {
 
    cv::medianBlur(*src->v, *dst->v, ksize);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmerge(vector_Mat* mv, Mat_t* dst)
{
try {
 
    cv::merge(*mv, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmin(Mat_t* src1, Mat_t* src2, Mat_t* dst)
{
try {
 
    cv::min(*src1->v, *src2->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(RotatedRect_t*)   pCvminAreaRect(Mat_t* points)
{
    RotatedRect_t*  retval = new RotatedRect_t;
try { 
    RotatedRect tr = cv::minAreaRect(*points->v);
    RotatedRect*  t = new RotatedRect();
    retval->v = t;
    RotatedRect_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvminEnclosingCircle(Mat_t* points, Point2f_t* center, float radius)
{
try {
 
    cv::minEnclosingCircle(*points->v, *center->v, radius);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvminMaxLoc(Mat_t* src, double* minVal, double* maxVal, Point_t* minLoc, Point_t* maxLoc, Mat_t* mask)
{
try {
 
    cv::minMaxLoc(*src->v, minVal, maxVal, minLoc->v, maxLoc->v, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmixChannels(vector_Mat* src, vector_Mat* dst, vector_int* fromTo)
{
try {
 
    cv::mixChannels(*src, *dst, *fromTo);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Moments_t*)   pCvmoments(Mat_t* _array, bool binaryImage)
{
    Moments_t*  retval = new Moments_t;
try { 
    Moments tr = cv::moments(*_array->v, binaryImage);
    Moments*  t = new Moments();
    retval->v = t;
    Moments_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvmorphologyEx(Mat_t* src, Mat_t* dst, int op, Mat_t* kernel, Point_t* anchor, int iterations, int borderType, Scalar_t* borderValue)
{
try {
 
    cv::morphologyEx(*src->v, *dst->v, op, *kernel->v, *anchor->v, iterations, borderType, *borderValue->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmoveWindow(string_t* winname, int x, int y)
{
try {
 
    cv::moveWindow(winname->v, x, y);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmulSpectrums(Mat_t* a, Mat_t* b, Mat_t* c, int flags, bool conjB)
{
try {
 
    cv::mulSpectrums(*a->v, *b->v, *c->v, flags, conjB);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmulTransposed(Mat_t* src, Mat_t* dst, bool aTa, Mat_t* delta, double scale, int dtype)
{
try {
 
    cv::mulTransposed(*src->v, *dst->v, aTa, *delta->v, scale, dtype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvmultiply(Mat_t* src1, Mat_t* src2, Mat_t* dst, double scale, int dtype)
{
try {
 
    cv::multiply(*src1->v, *src2->v, *dst->v, scale, dtype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvnamedWindow(string_t* winname, int flags)
{
try {
 
    cv::namedWindow(winname->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvnorm(Mat_t* src1, int normType, Mat_t* mask)
{
    double retval;
try {
 
    retval = cv::norm(*src1->v, normType, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(double)   pCvnorm2(Mat_t* src1, Mat_t* src2, int normType, Mat_t* mask)
{
    double retval;
try {
 
    retval = cv::norm(*src1->v, *src2->v, normType, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvnormalize(Mat_t* src, Mat_t* dst, double alpha, double beta, int norm_type, int dtype, Mat_t* mask)
{
try {
 
    cv::normalize(*src->v, *dst->v, alpha, beta, norm_type, dtype, *mask->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvpatchNaNs(Mat_t* a, double val)
{
try {
 
    cv::patchNaNs(*a->v, val);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvperspectiveTransform(Mat_t* src, Mat_t* dst, Mat_t* m)
{
try {
 
    cv::perspectiveTransform(*src->v, *dst->v, *m->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvphase(Mat_t* x, Mat_t* y, Mat_t* angle, bool angleInDegrees)
{
try {
 
    cv::phase(*x->v, *y->v, *angle->v, angleInDegrees);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Point2d_t*)   pCvphaseCorrelate(Mat_t* src1, Mat_t* src2, Mat_t* window)
{
    Point2d_t*  retval = new Point2d_t;
try { 
    Point2d tr = cv::phaseCorrelate(*src1->v, *src2->v, *window->v);
    Point2d*  t = new Point2d();
    retval->v = t;
    Point2d_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Point2d_t*)   pCvphaseCorrelateRes(Mat_t* src1, Mat_t* src2, Mat_t* window, double* response)
{
    Point2d_t*  retval = new Point2d_t;
try { 
    Point2d tr = cv::phaseCorrelateRes(*src1->v, *src2->v, *window->v, response);
    Point2d*  t = new Point2d();
    retval->v = t;
    Point2d_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(double)   pCvpointPolygonTest(Mat_t* contour, Point2f_t* pt, bool measureDist)
{
    double retval;
try {
 
    retval = cv::pointPolygonTest(*contour->v, *pt->v, measureDist);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvpolarToCart(Mat_t* magnitude, Mat_t* angle, Mat_t* x, Mat_t* y, bool angleInDegrees)
{
try {
 
    cv::polarToCart(*magnitude->v, *angle->v, *x->v, *y->v, angleInDegrees);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvpolylines(Mat_t* img, vector_Mat* pts, bool isClosed, Scalar_t* color, int thickness, int lineType, int shift)
{
try {
 
    cv::polylines(*img->v, *pts, isClosed, *color->v, thickness, lineType, shift);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvpow(Mat_t* src, double power, Mat_t* dst)
{
try {
 
    cv::pow(*src->v, power, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvpreCornerDetect(Mat_t* src, Mat_t* dst, int ksize, int borderType)
{
try {
 
    cv::preCornerDetect(*src->v, *dst->v, ksize, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvprojectPoints(Mat_t* objectPoints, Mat_t* rvec, Mat_t* tvec, Mat_t* cameraMatrix, Mat_t* distCoeffs, Mat_t* imagePoints, Mat_t* jacobian, double aspectRatio)
{
try {
 
    cv::projectPoints(*objectPoints->v, *rvec->v, *tvec->v, *cameraMatrix->v, *distCoeffs->v, *imagePoints->v, *jacobian->v, aspectRatio);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvputText(Mat_t* img, string_t* text, Point_t* org, int fontFace, double fontScale, Scalar_t* color, int thickness, int lineType, bool bottomLeftOrigin)
{
try {
 
    cv::putText(*img->v, text->v, *org->v, fontFace, fontScale, *color->v, thickness, lineType, bottomLeftOrigin);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvpyrDown(Mat_t* src, Mat_t* dst, Size_t* dstsize, int borderType)
{
try {
 
    cv::pyrDown(*src->v, *dst->v, *dstsize->v, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvpyrMeanShiftFiltering(Mat_t* src, Mat_t* dst, double sp, double sr, int maxLevel, TermCriteria_t* termcrit)
{
try {
 
    cv::pyrMeanShiftFiltering(*src->v, *dst->v, sp, sr, maxLevel, *termcrit->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvpyrUp(Mat_t* src, Mat_t* dst, Size_t* dstsize, int borderType)
{
try {
 
    cv::pyrUp(*src->v, *dst->v, *dstsize->v, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvrandShuffle(Mat_t* dst, double iterFactor)
{
try {
 
    cv::randShuffle_(*dst->v, iterFactor);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvrandn(Mat_t* dst, Mat_t* mean, Mat_t* stddev)
{
try {
 
    cv::randn(*dst->v, *mean->v, *stddev->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvrandu(Mat_t* dst, Mat_t* low, Mat_t* high)
{
try {
 
    cv::randu(*dst->v, *low->v, *high->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvrectangle(Mat_t* img, Point_t* pt1, Point_t* pt2, Scalar_t* color, int thickness, int lineType, int shift)
{
try {
 
    cv::rectangle(*img->v, *pt1->v, *pt2->v, *color->v, thickness, lineType, shift);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(float)   pCvrectify3Collinear(Mat_t* cameraMatrix1, Mat_t* distCoeffs1, Mat_t* cameraMatrix2, Mat_t* distCoeffs2, Mat_t* cameraMatrix3, Mat_t* distCoeffs3, vector_Mat* imgpt1, vector_Mat* imgpt3, Size_t* imageSize, Mat_t* R12, Mat_t* T12, Mat_t* R13, Mat_t* T13, Mat_t* R1, Mat_t* R2, Mat_t* R3, Mat_t* P1, Mat_t* P2, Mat_t* P3, Mat_t* Q, double alpha, Size_t* newImgSize, Rect_t* roi1, Rect_t* roi2, int flags)
{
    float retval;
try {
 
    retval = cv::rectify3Collinear(*cameraMatrix1->v, *distCoeffs1->v, *cameraMatrix2->v, *distCoeffs2->v, *cameraMatrix3->v, *distCoeffs3->v, *imgpt1, *imgpt3, *imageSize->v, *R12->v, *T12->v, *R13->v, *T13->v, *R1->v, *R2->v, *R3->v, *P1->v, *P2->v, *P3->v, *Q->v, alpha, *newImgSize->v, roi1->v, roi2->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvreduce(Mat_t* src, Mat_t* dst, int dim, int rtype, int dtype)
{
try {
 
    cv::reduce(*src->v, *dst->v, dim, rtype, dtype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvremap(Mat_t* src, Mat_t* dst, Mat_t* map1, Mat_t* map2, int interpolation, int borderMode, Scalar_t* borderValue)
{
try {
 
    cv::remap(*src->v, *dst->v, *map1->v, *map2->v, interpolation, borderMode, *borderValue->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvrepeat(Mat_t* src, int ny, int nx, Mat_t* dst)
{
try {
 
    cv::repeat(*src->v, ny, nx, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvreprojectImageTo3D(Mat_t* disparity, Mat_t* _3dImage, Mat_t* Q, bool handleMissingValues, int ddepth)
{
try {
 
    cv::reprojectImageTo3D(*disparity->v, *_3dImage->v, *Q->v, handleMissingValues, ddepth);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvresize(Mat_t* src, Mat_t* dst, Size_t* dsize, double fx, double fy, int interpolation)
{
try {
 
    cv::resize(*src->v, *dst->v, *dsize->v, fx, fy, interpolation);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvresizeWindow(string_t* winname, int width, int height)
{
try {
 
    cv::resizeWindow(winname->v, width, height);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvscaleAdd(Mat_t* src1, double alpha, Mat_t* src2, Mat_t* dst)
{
try {
 
    cv::scaleAdd(*src1->v, alpha, *src2->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsegmentMotion(Mat_t* mhi, Mat_t* segmask, vector_Rect* boundingRects, double timestamp, double segThresh)
{
try {
 
    cv::segmentMotion(*mhi->v, *segmask->v, *boundingRects, timestamp, segThresh);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsepFilter2D(Mat_t* src, Mat_t* dst, int ddepth, Mat_t* kernelX, Mat_t* kernelY, Point_t* anchor, double delta, int borderType)
{
try {
 
    cv::sepFilter2D(*src->v, *dst->v, ddepth, *kernelX->v, *kernelY->v, *anchor->v, delta, borderType);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsetIdentity(Mat_t* mtx, Scalar_t* s)
{
try {
 
    cv::setIdentity(*mtx->v, *s->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsetNumThreads(int nthreads)
{
try {
 
    cv::setNumThreads(nthreads);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsetRNGSeed(int seed)
{
try {
 
    cv::setRNGSeed(seed);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsetTrackbarPos(string_t* trackbarname, string_t* winname, int pos)
{
try {
 
    cv::setTrackbarPos(trackbarname->v, winname->v, pos);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsetUseOptimized(bool onoff)
{
try {
 
    cv::setUseOptimized(onoff);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsetWindowProperty(string_t* winname, int prop_id, double prop_value)
{
try {
 
    cv::setWindowProperty(winname->v, prop_id, prop_value);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvsolve(Mat_t* src1, Mat_t* src2, Mat_t* dst, int flags)
{
    bool retval;
try {
 
    retval = cv::solve(*src1->v, *src2->v, *dst->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(int)   pCvsolveCubic(Mat_t* coeffs, Mat_t* roots)
{
    int retval;
try {
 
    retval = cv::solveCubic(*coeffs->v, *roots->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(bool)   pCvsolvePnP(Mat_t* objectPoints, Mat_t* imagePoints, Mat_t* cameraMatrix, Mat_t* distCoeffs, Mat_t* rvec, Mat_t* tvec, bool useExtrinsicGuess, int flags)
{
    bool retval;
try {
 
    retval = cv::solvePnP(*objectPoints->v, *imagePoints->v, *cameraMatrix->v, *distCoeffs->v, *rvec->v, *tvec->v, useExtrinsicGuess, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvsolvePnPRansac(Mat_t* objectPoints, Mat_t* imagePoints, Mat_t* cameraMatrix, Mat_t* distCoeffs, Mat_t* rvec, Mat_t* tvec, bool useExtrinsicGuess, int iterationsCount, float reprojectionError, int minInliersCount, Mat_t* inliers, int flags)
{
try {
 
    cv::solvePnPRansac(*objectPoints->v, *imagePoints->v, *cameraMatrix->v, *distCoeffs->v, *rvec->v, *tvec->v, useExtrinsicGuess, iterationsCount, reprojectionError, minInliersCount, *inliers->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(double)   pCvsolvePoly(Mat_t* coeffs, Mat_t* roots, int maxIters)
{
    double retval;
try {
 
    retval = cv::solvePoly(*coeffs->v, *roots->v, maxIters);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvsort(Mat_t* src, Mat_t* dst, int flags)
{
try {
 
    cv::sort(*src->v, *dst->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsortIdx(Mat_t* src, Mat_t* dst, int flags)
{
try {
 
    cv::sortIdx(*src->v, *dst->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsplit(Mat_t* m, vector_Mat* mv)
{
try {
 
    cv::split(*m->v, *mv);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvsqrt(Mat_t* src, Mat_t* dst)
{
try {
 
    cv::sqrt(*src->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvstartWindowThread()
{
    int retval;
try {
 
    retval = cv::startWindowThread();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(double)   pCvstereoCalibrate(vector_Mat* objectPoints, vector_Mat* imagePoints1, vector_Mat* imagePoints2, Mat_t* cameraMatrix1, Mat_t* distCoeffs1, Mat_t* cameraMatrix2, Mat_t* distCoeffs2, Size_t* imageSize, Mat_t* R, Mat_t* T, Mat_t* E, Mat_t* F, TermCriteria_t* criteria, int flags)
{
    double retval;
try {
 
    retval = cv::stereoCalibrate(*objectPoints, *imagePoints1, *imagePoints2, *cameraMatrix1->v, *distCoeffs1->v, *cameraMatrix2->v, *distCoeffs2->v, *imageSize->v, *R->v, *T->v, *E->v, *F->v, *criteria->v, flags);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvstereoRectify(Mat_t* cameraMatrix1, Mat_t* distCoeffs1, Mat_t* cameraMatrix2, Mat_t* distCoeffs2, Size_t* imageSize, Mat_t* R, Mat_t* T, Mat_t* R1, Mat_t* R2, Mat_t* P1, Mat_t* P2, Mat_t* Q, int flags, double alpha, Size_t* newImageSize, Rect_t* validPixROI1, Rect_t* validPixROI2)
{
try {
 
    cv::stereoRectify(*cameraMatrix1->v, *distCoeffs1->v, *cameraMatrix2->v, *distCoeffs2->v, *imageSize->v, *R->v, *T->v, *R1->v, *R2->v, *P1->v, *P2->v, *Q->v, flags, alpha, *newImageSize->v, validPixROI1->v, validPixROI2->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvstereoRectifyUncalibrated(Mat_t* points1, Mat_t* points2, Mat_t* F, Size_t* imgSize, Mat_t* H1, Mat_t* H2, double threshold)
{
    bool retval;
try {
 
    retval = cv::stereoRectifyUncalibrated(*points1->v, *points2->v, *F->v, *imgSize->v, *H1->v, *H2->v, threshold);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvsubtract(Mat_t* src1, Mat_t* src2, Mat_t* dst, Mat_t* mask, int dtype)
{
try {
 
    cv::subtract(*src1->v, *src2->v, *dst->v, *mask->v, dtype);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(Scalar_t*)   pCvsumElems(Mat_t* src)
{
    Scalar_t*  retval = new Scalar_t;
try { 
    Scalar tr = cv::sum(*src->v);
    Scalar*  t = new Scalar();
    retval->v = t;
    Scalar_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(double)   pCvthreshold(Mat_t* src, Mat_t* dst, double thresh, double maxval, int _type)
{
    double retval;
try {
 
    retval = cv::threshold(*src->v, *dst->v, thresh, maxval, _type);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(Scalar_t*)   pCvtrace(Mat_t* mtx)
{
    Scalar_t*  retval = new Scalar_t;
try { 
    Scalar tr = cv::trace(*mtx->v);
    Scalar*  t = new Scalar();
    retval->v = t;
    Scalar_cpy(retval, tr);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvtransform(Mat_t* src, Mat_t* dst, Mat_t* m)
{
try {
 
    cv::transform(*src->v, *dst->v, *m->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvtranspose(Mat_t* src, Mat_t* dst)
{
try {
 
    cv::transpose(*src->v, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvtriangulatePoints(Mat_t* projMatr1, Mat_t* projMatr2, Mat_t* projPoints1, Mat_t* projPoints2, Mat_t* points4D)
{
try {
 
    cv::triangulatePoints(*projMatr1->v, *projMatr2->v, *projPoints1->v, *projPoints2->v, *points4D->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvundistort(Mat_t* src, Mat_t* dst, Mat_t* cameraMatrix, Mat_t* distCoeffs, Mat_t* newCameraMatrix)
{
try {
 
    cv::undistort(*src->v, *dst->v, *cameraMatrix->v, *distCoeffs->v, *newCameraMatrix->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvundistortPoints(Mat_t* src, Mat_t* dst, Mat_t* cameraMatrix, Mat_t* distCoeffs, Mat_t* R, Mat_t* P)
{
try {
 
    cv::undistortPoints(*src->v, *dst->v, *cameraMatrix->v, *distCoeffs->v, *R->v, *P->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvupdateMotionHistory(Mat_t* silhouette, Mat_t* mhi, double timestamp, double duration)
{
try {
 
    cv::updateMotionHistory(*silhouette->v, *mhi->v, timestamp, duration);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(bool)   pCvuseOptimized()
{
    bool retval;
try {
 
    retval = cv::useOptimized();
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvvalidateDisparity(Mat_t* disparity, Mat_t* cost, int minDisparity, int numberOfDisparities, int disp12MaxDisp)
{
try {
 
    cv::validateDisparity(*disparity->v, *cost->v, minDisparity, numberOfDisparities, disp12MaxDisp);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvvconcat(vector_Mat* src, Mat_t* dst)
{
try {
 
    cv::vconcat(*src, *dst->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(int)   pCvwaitKey(int delay)
{
    int retval;
try {
 
    retval = cv::waitKey(delay);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return (retval);

}
CVAPI(void)   pCvwarpAffine(Mat_t* src, Mat_t* dst, Mat_t* M, Size_t* dsize, int flags, int borderMode, Scalar_t* borderValue)
{
try {
 
    cv::warpAffine(*src->v, *dst->v, *M->v, *dsize->v, flags, borderMode, *borderValue->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvwarpPerspective(Mat_t* src, Mat_t* dst, Mat_t* M, Size_t* dsize, int flags, int borderMode, Scalar_t* borderValue)
{
try {
 
    cv::warpPerspective(*src->v, *dst->v, *M->v, *dsize->v, flags, borderMode, *borderValue->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
CVAPI(void)   pCvwatershed(Mat_t* image, Mat_t* markers)
{
try {
 
    cv::watershed(*image->v, *markers->v);
} catch (std::exception &e) {
      exceptionDisplay(e.what());
};

    return;

}
