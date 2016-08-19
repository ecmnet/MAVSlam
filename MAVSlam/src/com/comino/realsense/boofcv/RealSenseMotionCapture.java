package com.comino.realsense.boofcv;

import org.mavlink.messages.lquac.msg_msp_mocap;
import org.mavlink.messages.lquac.msg_msp_status;

import com.comino.mav.control.IMAVMSPController;
import com.comino.realsense.boofcv.StreamRealSenseVisDepth.Listener;
import com.comino.realsense.boofcv.odometry.FactoryRealSenseOdometry;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.sfm.d3.DepthVisualOdometry;
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

public class RealSenseMotionCapture {

	private StreamRealSenseVisDepth realsense;
	private RealSenseInfo info;
	private DepthVisualOdometry<GrayU8,GrayU16> visualOdometry;

	private long oldTimeDepth=0;

	private Vector3D_F64 pos;
	private Vector3D_F64 pos_old;
	private Vector3D_F64 speed = new Vector3D_F64();

	private long tms=0;

	public RealSenseMotionCapture(IMAVMSPController control) {

		info = new RealSenseInfo(320,240, RealSenseInfo.MODE_RGB);

		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{1, 2, 4, 8};
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<GrayU8> tracker =
				FactoryPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(100, 2, 1),
						GrayU8.class, GrayS16.class);

		DepthSparse3D<GrayU16> sparseDepth = new DepthSparse3D.I<GrayU16>(1e-3);

		try {
			realsense = new StreamRealSenseVisDepth(0,info);
		} catch(Exception e) {
			System.out.println(e.getMessage());
		}

		visualOdometry = FactoryRealSenseOdometry.depthDepthPnP(1.2, 120, 2, 200, 50, true,
				sparseDepth, tracker, GrayU8.class, GrayU16.class);

		visualOdometry.setCalibration(realsense.getIntrinsics(),new DoNothingPixelTransform_F32());

		realsense.registerListener(new Listener() {

			float fps; float dt; float md; int mf=0; int fpm;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {

				dt = (timeRgb - oldTimeDepth)/1000f;

				if((System.currentTimeMillis() - tms) > 250) {
					tms = System.currentTimeMillis();
					if(mf>0)
					  fps = fpm/mf;
					md = 0; mf=0; fpm=0;
				}
				mf++;
				fpm += (int)(1f/((timeRgb - oldTimeDepth)/1000f)+0.5f);
				oldTimeDepth = timeRgb;

				if( !visualOdometry.process(rgb.getBand(0),depth) ) {
					pos = null; pos_old = null; speed.x=0; speed.y=0; speed.z=0;
					visualOdometry.reset();
					msg_msp_mocap msg = new msg_msp_mocap(1,2);
					msg.vx = (float) speed.x;
					msg.vy = (float) speed.z;
					msg.vz = (float) speed.y;
					msg.flags = 0;
					msg.tms = System.nanoTime() / 1000;
					control.sendMAVLinkMessage(msg);
					return;
				}

				Se3_F64 leftToWorld = visualOdometry.getCameraToWorld();
				pos = leftToWorld.getT();

				if(pos_old!=null) {
					speed.x = (speed.x + (pos.x - pos_old.x)/dt)/2;
					speed.y = (speed.y + (pos.y - pos_old.y)/dt)/2;
					speed.z = (speed.z + (pos.z - pos_old.z)/dt)/2;
				}

				if(control!=null) {
					if(Math.abs(speed.x)< 10 && Math.abs(speed.y)< 10 && Math.abs(speed.z)< 10 ) {
						msg_msp_mocap msg = new msg_msp_mocap(1,2);
						msg.vx = (float) speed.x;
						msg.vy = (float) speed.z;
						msg.vz = (float) speed.y;
						msg.fps = fps;
						msg.flags = msg.flags | 1;
						msg.tms = System.nanoTime() / 1000;
						control.sendMAVLinkMessage(msg);
					}
				}

				pos_old = pos.copy();
			}
		}).start();

	}

	public RealSenseMotionCapture() {
		this(null);
	}


	public static void main(String[] args) {
		new RealSenseMotionCapture();
	}

}
