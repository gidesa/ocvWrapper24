#pragma once

CVAPI(void) pCvDrawMatches(Mat_t* img1, vector_KeyPoint* keypoints1,
	Mat_t* img2, vector_KeyPoint* keypoints2,
	vector_DMatch* matches1to2, Mat_t* outImg, Scalar_t* matchColor, Scalar_t* singlePointColor)
	//const vector<char>& matchesMask = vector<char>(), int flags = DrawMatchesFlags::DEFAULT) 
{
try {
	cv::drawMatches( *img1->v, *keypoints1, *img2->v, *keypoints2, *matches1to2, *outImg->v, *matchColor->v,  *singlePointColor->v);
	//matchesMask, flags);
} catch (Exception e) {
      exceptionDisplay(e.msg);
};
}

CVAPI(void)   pCvPCACompute2(Mat_t* data, Mat_t* mean, Mat_t* eigenvectors, Mat_t* eigenvalues, int maxComponents)
{
	try {
		PCA pca;
		pca(*data->v, *mean->v, 0, maxComponents);
		pca.mean.copyTo(*mean->v);
		pca.eigenvectors.copyTo(*eigenvectors->v);
		pca.eigenvalues.copyTo(*eigenvalues->v);
	}
	catch (Exception e) {
		exceptionDisplay(e.msg);
	};
}


