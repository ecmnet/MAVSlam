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

package com.comino.realsense.boofcv;


import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_vision_position_estimate;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.model.DataModel;
import com.comino.msp.model.segment.LogMessage;
import com.comino.msp.model.segment.Status;
import com.comino.msp.utils.MSPMathUtils;
import com.comino.realsense.boofcv.StreamRealSenseVisDepth.Listener;
import com.comino.realsense.boofcv.odometry.FactoryRealSenseOdometry;
import com.comino.realsense.boofcv.odometry.RealSenseDepthVisualOdometry;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.alg.distort.DoNothingPixelTransform_F32;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.alg.tracker.klt.PkltConfig;
import boofcv.factory.feature.tracker.FactoryPointTrackerTwoPass;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class RealSensePositionEstimator {

	private static final int MIN_COUNT   = 10;
	private static final int MIN_QUALITY = 20;
	private static final int MAXTRACKS   = 120;

	private StreamRealSenseVisDepth realsense;
	private RealSenseInfo info;
	private RealSenseDepthVisualOdometry<GrayU8,GrayU16> visualOdometry;

	private long oldTimeDepth=0;

	private Vector3D_F64 pos_raw;
	private Vector3D_F64 pos_raw_old;
	private Vector3D_F64 speed   = new Vector3D_F64();
	private Vector3D_F64 pos     = new Vector3D_F64();

	private long tms=0;
	private long framecount=0;

	private DataModel model;

	private boolean isRunning = false;
	private IMAVMSPController control;

	private float init_head_rad = 0;
	private AccessPointTracks3D points;

	public RealSensePositionEstimator(IMAVMSPController control) {
        this.control = control;
		this.model = control.getCurrentModel();

		info = new RealSenseInfo(320,240, RealSenseInfo.MODE_RGB);
	//	info = new RealSenseInfo(649,480, RealSenseInfo.MODE_RGB);

		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{1, 2, 4, 8};
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<GrayU8> tracker =
				FactoryPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(MAXTRACKS, 2, 1),
						GrayU8.class, GrayS16.class);

		DepthSparse3D<GrayU16> sparseDepth = new DepthSparse3D.I<GrayU16>(1e-3);

		try {
			realsense = new StreamRealSenseVisDepth(0,info);
		} catch(Exception e) {

		}

		visualOdometry = FactoryRealSenseOdometry.depthDepthPnP(1.2, 120, 2, 200, 50, true,
				sparseDepth, tracker, GrayU8.class, GrayU16.class);

		visualOdometry.setCalibration(realsense.getIntrinsics(),new DoNothingPixelTransform_F32());

		this.points = (AccessPointTracks3D)visualOdometry;

		realsense.registerListener(new Listener() {

			float fps; float dt; int mf=0; int fpm; float[] pos_rot = new float[2]; int quality=0;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {

				framecount++;

				dt = (timeDepth - oldTimeDepth)/1000f;

				if((System.currentTimeMillis() - tms) > 250) {
					tms = System.currentTimeMillis();
					if(mf>0)
						fps = fpm/mf;
					mf=0; fpm=0;
				}
				mf++;
				fpm += (int)(1f/((timeDepth - oldTimeDepth)/1000f)+0.5f);
				oldTimeDepth = timeDepth;

				if( !visualOdometry.process(rgb.getBand(0),depth) ) {
					init();
					return;
				}

				Se3_F64 leftToWorld = visualOdometry.getCameraToWorld();
				pos_raw = leftToWorld.getT();


				if(pos_raw_old!=null) {

					speed.y = (pos_raw.x - pos_raw_old.x)/dt;
					speed.x = (pos_raw.z - pos_raw_old.z)/dt;
					speed.z = (pos_raw.y - pos_raw_old.y)/dt;

					if(Math.abs(speed.x)< 20 && Math.abs(speed.y)< 20 && Math.abs(speed.z)< 20 ) {
						pos.x += speed.x * dt;
						pos.y += speed.y * dt;
						pos.z += speed.z * dt;
					} else {
						init();
						return;
					}
				}

				quality = visualOdometry.getInlierCount() *100 / MAXTRACKS;

				if(framecount < MIN_COUNT) {
					MSPMathUtils.rotateRad(pos_rot, model.state.l_x,model.state.l_y,init_head_rad);
					pos.set(pos_rot[0],pos_rot[1], model.raw.di);
					return;
				} else {
					if(quality < MIN_QUALITY) {
						init();
					    return;
					}
				}

				if(control!=null) {

    				MSPMathUtils.rotateRad(pos_rot,(float)pos.x,(float)pos.y,-init_head_rad);

					msg_vision_position_estimate sms = new msg_vision_position_estimate(1,1);
					sms.usec =timeDepth*1000;
					sms.x = (float) pos_rot[0];
					sms.y = (float) pos_rot[1];
					sms.z = (float) pos.z;
					control.sendMAVLinkMessage(sms);


					msg_msp_vision msg = new msg_msp_vision(1,2);
					msg.x = (float) pos_rot[0];
					msg.y = (float) pos_rot[1];
					msg.z = (float) pos.z;
					msg.vx = (float) speed.x;
					msg.vy = (float) speed.y;
					msg.vz = (float) speed.z;
					msg.h = model.state.h;
					msg.quality = quality;
					msg.fps = fps;
					msg.flags = msg.flags | 1;
					msg.tms = System.nanoTime() / 1000;
					control.sendMAVLinkMessage(msg);
				}

				pos_raw_old = pos_raw.copy();
			}
		});
	}

	public RealSensePositionEstimator() {
		this(null);
	}

	public void start() {
		isRunning = true;
		init();
		if(realsense!=null)
		    realsense.start();
	}

	public boolean isRunning() {
		return isRunning;
	}

	private void init() {
		init_head_rad = MSPMathUtils.toRad(model.state.h);
		control.writeLogMessage(new LogMessage("[vis] reset odometry",MAV_SEVERITY.MAV_SEVERITY_INFO));
		visualOdometry.reset();
		pos_raw = null; pos_raw_old = null; speed.x=0; speed.y=0; speed.z=0;
		framecount = 0;
	}

	private void collisionDetection() {

	}


	public static void main(String[] args) {
		new RealSensePositionEstimator();
	}

}
