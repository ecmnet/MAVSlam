package com.comino.slam.detectors.impl;


import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.model.DataModel;
import com.comino.msp.model.segment.LogMessage;
import com.comino.realsense.boofcv.odometry.RealSenseDepthVisualOdometry;
import com.comino.slam.detectors.ISLAMDetector;
import com.comino.slam.detectors.space.Feature;
import com.comino.slam.detectors.space.Space;
import com.comino.slam.model.RotationModel;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.Color3_I32;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point3D_F64;

/****************************************************************************
*
*   Copyright (c) 2016 Eike Mansfeld ecm@gmx.de. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

public class SegmentedCollisionDetector implements ISLAMDetector {



	private IMAVMSPController control;
	private DataModel         model;
	private Space   obstacles;

	public SegmentedCollisionDetector(IMAVMSPController control) {
		this.control  = control;
		this.model    = control.getCurrentModel();
		this.obstacles = new Space();
	}

	@Override
	public void process(RealSenseDepthVisualOdometry<GrayU8,GrayU16> odometry, GrayU16 depth, Planar<GrayU8> rgb, RotationModel att) {
		int x = 0; int y = 0; int z = 0;

		if(att.quality < 10)
			return;

		obstacles.clear();
		obstacles.setOrigin(model.state.l_x, model.state.l_y, model.state.l_z);

		for( int i = 0; i < odometry.getInlierCount(); i++ ) {

				Point3D_F64 point = odometry.getTrackLocation(i);
				obstacles.addFeature(new Feature(point.x,point.y,point.z));


		}

	   Point3D_F32 obstacle_pos = obstacles.getMaxFeaturesPositionWorld();
	   if(obstacle_pos!=null)
		   control.writeLogMessage(
			  new LogMessage("[slam] Possible collision at ("+obstacle_pos.x+","+obstacle_pos.y+","+obstacle_pos.z+")",
					MAV_SEVERITY.MAV_SEVERITY_CRITICAL));

	}

}
