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

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.model.DataModel;
import com.comino.realsense.boofcv.odometry.RealSenseDepthVisualOdometry;
import com.comino.server.mjpeg.MJPEGHandler;
import com.comino.slam.detectors.ISLAMDetector;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class SimpleCollisionDetector implements ISLAMDetector {

	private static final float     MIN_DISTANCE_M         = 0.75f;
	private static final int       COLLISION_FACTOR       = 20;

	private IMAVMSPController control;

	private int center_x=0;
	private int center_y=0;
	private DataModel model;

	public SimpleCollisionDetector(IMAVMSPController control, MJPEGHandler streamer) {
		this.control = control;
		this.model   = control.getCurrentModel();
		streamer.registerOverlayListener(ctx -> {
           if(center_x > 0 && center_y > 0)
        	   ctx.drawRect(center_x-20, center_y-20, 40, 40);
		});
	}

	@Override
	public void process(RealSenseDepthVisualOdometry<GrayU8,GrayU16> odometry, GrayU16 depth, Planar<GrayU8> rgb) {
	int count=0; int x = 0; int y = 0; int cx=0; int cy=0; float distance=0; double dx,dy,dz;



		AccessPointTracks3D points = (AccessPointTracks3D)odometry;

		count = 0; cx = 0; cy = 0;
		for( int i = 0; i < points.getAllTracks().size(); i++ ) {
			if(points.isInlier(i)) {
				x = (int)points.getAllTracks().get(i).x;
				y = (int)points.getAllTracks().get(i).y;

				// TODO: Distance determination not working (rotate into bodyframe)
				// Problem: get Trackposition in NED Frame => rotate with RoTMatrix

				if(y < depth.height/2) {

					dx = odometry.getTrackLocation(i).x - model.state.l_x;
					dy = odometry.getTrackLocation(i).y - model.state.l_y;
					dz = odometry.getTrackLocation(i).z - model.state.l_z;

					distance = (float)Math.sqrt(dx*dx + dy*dy + dz*dz);

					if(distance<MIN_DISTANCE_M) {
						cx += x; cy +=y;
						count++;
					}
				}
			}
		}

		if(count>COLLISION_FACTOR) {
			center_x = cx / count; center_y = cy / count;

		} else {
			center_x = 0; center_y = 0;
		}
	}
}
