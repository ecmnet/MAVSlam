/*
 * Copyright (c) 2011-2016, Peter Abeles, Eike Mansfeld. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.comino.slam.boofcv.vio;

import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.fitting.modelset.ransac.Ransac;

import com.comino.slam.boofcv.vio.odometry.MAVOdomPixelDepthPnPVIO;
import com.comino.slam.boofcv.vio.odometry.MAVOdomPixelDepthPnP_to_DepthVisualOdometryVIO;
import com.comino.slam.boofcv.vo.odometry.MAVOdomPixelDepthPnP;

import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.geo.Estimate1ofPnP;
import boofcv.abst.geo.RefinePnP;
import boofcv.abst.sfm.DepthSparse3D_to_PixelTo3D;
import boofcv.abst.sfm.ImagePixelTo3D;
import boofcv.alg.geo.DistanceModelMonoPixels;
import boofcv.alg.geo.pose.PnPDistanceReprojectionSq;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.factory.geo.EnumPNP;
import boofcv.factory.geo.EstimatorToGenerator;
import boofcv.factory.geo.FactoryMultiView;
import boofcv.struct.geo.Point2D3D;
import boofcv.struct.image.ImageGray;
import boofcv.struct.image.ImageType;
import georegression.fitting.se.ModelManagerSe3_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

/**
 * Factory for creating visual odometry algorithms.
 *
 * @author Peter Abeles
 */
public class FactoryMAVOdometryVIO {



	/**
	 * Depth sensor based visual odometry algorithm which runs a sparse feature tracker in the visual camera and
	 * estimates the range of tracks once when first detected using the depth sensor.
	 *
	 * @see MAVOdomPixelDepthPnP
	 *
	 * @param thresholdAdd Add new tracks when less than this number are in the inlier set.  Tracker dependent. Set to
	 *                     a value &le; 0 to add features every frame.
	 * @param thresholdRetire Discard a track if it is not in the inlier set after this many updates.  Try 2
	 * @param sparseDepth Extracts depth of pixels from a depth sensor.
	 * @param visualType Type of visual image being processed.
	 * @param depthType Type of depth image being processed.
	 * @return StereoVisualOdometry
	 */
	public static <Vis extends ImageGray, Depth extends ImageGray>
	MAVOdomPixelDepthPnP_to_DepthVisualOdometryVIO<Vis, Depth> depthPnP(double inlierPixelTol,
												 int thresholdAdd,
												 int thresholdRetire ,
												 int ransacIterations ,
												 int refineIterations ,
												 boolean doublePass ,
												 Point3D_F64 offset,
												 DepthSparse3D<Depth> sparseDepth,
												 PointTrackerTwoPass<Vis> tracker ,
												 Class<Vis> visualType , Class<Depth> depthType
												 ) {

		// Range from sparse disparity
		ImagePixelTo3D pixelTo3D = new DepthSparse3D_to_PixelTo3D<Depth>(sparseDepth);

		Estimate1ofPnP estimator = FactoryMultiView.computePnP_1(EnumPNP.P3P_FINSTERWALDER,-1,2);
		final DistanceModelMonoPixels<Se3_F64,Point2D3D> distance = new PnPDistanceReprojectionSq();

		ModelManagerSe3_F64 manager = new ModelManagerSe3_F64();
		EstimatorToGenerator<Se3_F64,Point2D3D> generator = new EstimatorToGenerator<Se3_F64,Point2D3D>(estimator);

		// 1/2 a pixel tolerance for RANSAC inliers
		double ransacTOL = inlierPixelTol * inlierPixelTol;

		ModelMatcher<Se3_F64, Point2D3D> motion =
				new Ransac<Se3_F64, Point2D3D>(2323, manager, generator, distance, ransacIterations, ransacTOL);

		RefinePnP refine = null;

		if( refineIterations > 0 ) {
			refine = FactoryMultiView.refinePnP(1e-12,refineIterations);
		}

		MAVOdomPixelDepthPnPVIO<Vis> alg = new MAVOdomPixelDepthPnPVIO<Vis>
						(thresholdAdd,thresholdRetire ,doublePass,offset, motion,pixelTo3D,refine,tracker,null,null);

		return new MAVOdomPixelDepthPnP_to_DepthVisualOdometryVIO<Vis,Depth>
				(sparseDepth,alg,distance, ImageType.single(visualType),depthType);
	}



}
