package com.comino.realsense.boofcv;


import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_vision_position_estimate;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.model.DataModel;
import com.comino.msp.model.segment.LogMessage;
import com.comino.msp.utils.MSPMathUtils;
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

	private Vector3D_F64 pos_raw;
	private Vector3D_F64 pos_raw_old;
	private Vector3D_F64 speed   = new Vector3D_F64();
	private Vector3D_F64 pos     = new Vector3D_F64();
	private Vector3D_F64 pos_ref = new Vector3D_F64();

	private long tms=0;
	private long framecount=0;
	private float  initial_heading =0;

	private DataModel model;

	private boolean isRunning = false;
	private IMAVMSPController control;

	public RealSenseMotionCapture(IMAVMSPController control) {
        this.control = control;
		this.model = control.getCurrentModel();

		info = new RealSenseInfo(320,240, RealSenseInfo.MODE_RGB);

		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{1, 2, 4, 8};
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<GrayU8> tracker =
				FactoryPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(200, 2, 1),
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

			float fps; float dt; float md; int mf=0; int fpm; float dh;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {
				framecount++;
				dt = (timeDepth - oldTimeDepth)/1000f;

				dh = MSPMathUtils.toRad(model.state.h) - initial_heading;

				if((System.currentTimeMillis() - tms) > 250) {
					tms = System.currentTimeMillis();
					if(mf>0)
						fps = fpm/mf;
					md = 0; mf=0; fpm=0;
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

					//					float[] sr = MSPMathUtils.rotateRad((float)((pos_raw.x - pos_old.x)/dt),
					//							         (float)((pos_raw.y - pos_old.y)/dt), -dh);

					speed.x = (pos_raw.x - pos_raw_old.x)/dt;
					speed.y = (pos_raw.z - pos_raw_old.z)/dt;
					speed.z = (pos_raw.y - pos_raw_old.y)/dt;

					if(Math.abs(speed.x)< 20 && Math.abs(speed.y)< 20 && Math.abs(speed.z)< 20 ) {
						pos.x += -speed.x * dt;
						pos.y += -speed.y * dt;
						pos.z +=  speed.z * dt;
					} else {
						init();
						return;
					}
				}

				if(control!=null) {

					msg_vision_position_estimate sms = new msg_vision_position_estimate(1,1);
					sms.usec =timeDepth*1000;
					sms.x = (float) pos.x;
					sms.y = (float) pos.y;
					sms.z = (float) pos.z;
					control.sendMAVLinkMessage(sms);


					msg_msp_vision msg = new msg_msp_vision(1,2);
					msg.x = (float) pos.x;
					msg.y = (float) pos.y;
					msg.z = (float) pos.z;
					msg.vx = (float) speed.x;
					msg.vy = (float) speed.y;
					msg.vz = (float) speed.z;
					msg.vh = dh;
					msg.fps = fps;
					msg.flags = msg.flags | 1;
					msg.tms = System.nanoTime() / 1000;
					control.sendMAVLinkMessage(msg);
				}

				pos_raw_old = pos_raw.copy();
			}
		});
	}

	public RealSenseMotionCapture() {
		this(null);
	}

	public void start() {
		isRunning = true;
		initial_heading = MSPMathUtils.toRad(model.state.h);
		init();
		realsense.start();
	}

	public boolean isRunning() {
		return isRunning;
	}

	private void init() {
		control.writeLogMessage(new LogMessage("[vis] reset odometry",MAV_SEVERITY.MAV_SEVERITY_INFO));
		visualOdometry.reset();
		pos_raw = null; pos_raw_old = null; speed.x=0; speed.y=0; speed.z=0;
		initial_heading = MSPMathUtils.toRad(model.state.h);
		pos.set(model.state.l_x,model.state.l_y, model.state.l_z);
		framecount = 0;
	}


	public static void main(String[] args) {
		new RealSenseMotionCapture();
	}

}
