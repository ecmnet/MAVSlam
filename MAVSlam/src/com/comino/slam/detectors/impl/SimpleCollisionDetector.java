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

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.model.segment.LogMessage;
import com.comino.realsense.boofcv.odometry.RealSenseDepthVisualOdometry;
import com.comino.server.mjpeg.impl.HttpMJPEGHandler;
import com.comino.slam.detectors.ISLAMDetector;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point3D_F64;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;

public class SimpleCollisionDetector implements ISLAMDetector {

	private static final float     MIN_DISTANCE_M         = 0.75f;

	private int center_x=0;
	private int center_y=0;

	private BooleanProperty collision = new SimpleBooleanProperty(false);

	private Point3D_F64 nearestPoint = new Point3D_F64();

	public SimpleCollisionDetector(IMAVMSPController control, HttpMJPEGHandler streamer) {
		streamer.registerOverlayListener(ctx -> {
			if(collision.get())
				ctx.fillOval(center_x-10, center_y-10, 20, 20);
		});

		collision.addListener((l,o,n) -> {
			if(n.booleanValue())
				control.writeLogMessage(new LogMessage("[vis] collision warning",
						MAV_SEVERITY.MAV_SEVERITY_WARNING));
			else
				control.writeLogMessage(new LogMessage("[vis] collision warning cleared",
						MAV_SEVERITY.MAV_SEVERITY_NOTICE));

		});
	}

	@Override
	public void process(RealSenseDepthVisualOdometry<GrayU8,GrayU16> odometry, GrayU16 depth, Planar<GrayU8> rgb) {
		int x = 0; int y = 0; float distance=Float.MAX_VALUE;

		AccessPointTracks3D points = (AccessPointTracks3D)odometry;

		for( int i = 0; i < points.getAllTracks().size(); i++ ) {
			if(points.isInlier(i)) {
				x = (int)points.getAllTracks().get(i).x;
				y = (int)points.getAllTracks().get(i).y;

				Point3D_F64 p = odometry.getTrackLocation(i);

				if(distance >  Math.abs(p.z)) {
					distance = (float)Math.abs(p.z);
					center_x = x; center_y = y;
					nearestPoint.set(p);
				}
			}
		}
		collision.set(nearestPoint.z <MIN_DISTANCE_M);
	}
}
