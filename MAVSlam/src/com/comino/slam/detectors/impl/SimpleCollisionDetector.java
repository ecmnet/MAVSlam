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

package com.comino.slam.detectors.impl;


import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.model.DataModel;
import com.comino.msp.model.segment.LogMessage;
import com.comino.msp.model.segment.Status;
import com.comino.realsense.boofcv.odometry.RealSenseDepthVisualOdometry;
import com.comino.server.mjpeg.impl.HttpMJPEGHandler;
import com.comino.slam.detectors.ISLAMDetector;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;

public class SimpleCollisionDetector implements ISLAMDetector {

	private static final float     MIN_DISTANCE_M         = 1.25f;

	private int center_x=0;
	private int center_y=0;

	private DataModel model = null;
	private Point3D_F64   pos   = new Point3D_F64();

	private BooleanProperty collision = new SimpleBooleanProperty(false);

	private List<NearestPoint> nearestPoints =  new ArrayList<NearestPoint>();

	public SimpleCollisionDetector(IMAVMSPController control, HttpMJPEGHandler streamer) {

		this.model = control.getCurrentModel();

		streamer.registerOverlayListener(ctx -> {
			if(collision.get()) {
				for(NearestPoint n : nearestPoints) {
					ctx.drawRect(n.plane_x-10, n.plane_y-10, 20, 20);
				}

				NearestPoint n = nearestPoints.get(0);
				ctx.drawString(String.format("Min.Distance: %#.2fm", n.p_body.z), 5, 20);

				ctx.drawOval(center_x-10, center_y-10, 20, 20);
				ctx.drawOval(center_x-15, center_y-15, 30, 30);
			}
		});

		collision.addListener((l,o,n) -> {
			if(n.booleanValue()) {
				control.writeLogMessage(new LogMessage("[vis] collision warning",
						MAV_SEVERITY.MAV_SEVERITY_WARNING));

			}
			else
				control.writeLogMessage(new LogMessage("[vis] collision warning cleared",
						MAV_SEVERITY.MAV_SEVERITY_NOTICE));

		});
	}

	/*
	 * This is a simple collision warner. It searches the closest inliner and presents a warning if the distance of this
	 * inline is < MIN_DISTANCE_M
	 */

	@Override
	public void process(RealSenseDepthVisualOdometry<GrayU8,GrayU16> odometry, GrayU16 depth, Planar<GrayU8> rgb) {
		int x = 0; int y = 0;

		AccessPointTracks3D points = (AccessPointTracks3D)odometry;

		nearestPoints.clear();

		if(points.getAllTracks().size()==0 || ( model.sys.isStatus(Status.MSP_LANDED) && model.hud.ar < 0.3f)) {
			collision.set(false);
			return;
		}

		center_x = 0; center_y = 0;

		for( int i = 0; i < points.getAllTracks().size(); i++ ) {
			if(points.isInlier(i)) {
				x = (int)points.getAllTracks().get(i).x;
				y = (int)points.getAllTracks().get(i).y;

				// p is the obstacle ccordinates in body-frame
				Point3D_F64 p = odometry.getTrackLocation(i);

				if(p.z < MIN_DISTANCE_M) {
					NearestPoint n = new NearestPoint();
					n.p_body = p;
					n.plane_x = x;
					n.plane_y = y;
					center_x = center_x + x;
					center_y = center_y + y;
					nearestPoints.add(n);

					// to get getWorld coordinates of p
					// SePointOps_F64.transform(odometry.getCameraToWorld(), p, pos);

					// TODO: get a 3D avoidance vector based on the obstacle points
					// TODO: Project to display frame and visualize the avoidance vector
					// TODO: Take over control of the vehicle and perform avoidance maneuver


				}
			}
		}
		if(nearestPoints.size()>1) {
			center_x = center_x / nearestPoints.size();
			center_y = center_y / nearestPoints.size();
			Collections.sort(nearestPoints, (a, b) -> {
				return Double.compare(a.p_body.z,b.p_body.z);
			});
			collision.set(true);
		} else
			collision.set(false);

	}

	private class NearestPoint {
		public Point3D_F64 p_body;
		public int plane_x;
		public int plane_y;
	}
}
