package com.comino.slam.detectors.impl;

import java.awt.Color;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.model.segment.LogMessage;
import com.comino.realsense.boofcv.odometry.RealSenseDepthVisualOdometry;
import com.comino.slam.detectors.ISLAMDetector;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class SimpleCollisionDetector implements ISLAMDetector {

	private static final int   MIN_DISTANCE_CM        = 800;
	private static final float COLLISION_FACTOR       = 0.6f;

	private IMAVMSPController control;
	private long tms = 0;

	public SimpleCollisionDetector(IMAVMSPController control) {
		this.control = control;

	}

	@Override
	public void process(RealSenseDepthVisualOdometry<GrayU8,GrayU16> odometry, GrayU16 depth, Planar<GrayU8> rgb, int quality) {
		int total = 0; float count = 0;int x = 0; int y = 0;

		if(quality < 10)
			return;

		AccessPointTracks3D points = (AccessPointTracks3D)odometry;

		for( int i = 0; i < points.getAllTracks().size(); i++ ) {
			if(points.isInlier(i)) {
				x = (int)points.getAllTracks().get(i).x;
				y = (int)points.getAllTracks().get(i).y;

				if(y < depth.height/2) {
					total++;
					if(depth.get(x,y)<MIN_DISTANCE_CM) count++;
				}
			}
		}

		if((count / total)>COLLISION_FACTOR && (System.currentTimeMillis()-tms)>1000) {
            tms = System.currentTimeMillis();
			control.writeLogMessage(new LogMessage("[vis] Collision warning "+(count/total),
					MAV_SEVERITY.MAV_SEVERITY_CRITICAL));

		}
	}
}
