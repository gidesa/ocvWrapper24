/* ocvWrapper: wrapper for C++ API  Opencv interface 
  Manually coded user functions

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


