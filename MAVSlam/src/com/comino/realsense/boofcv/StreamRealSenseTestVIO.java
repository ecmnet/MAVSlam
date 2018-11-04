/****************************************************************************
 *
 *   Copyright (c) 2017 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import com.comino.mq.bus.MWMessageBus;
import com.comino.mq.tests.OptPos;
import com.comino.msp.utils.MSPMathUtils;
import com.comino.realsense.boofcv.StreamRealSenseVisDepth.Listener;
import com.comino.slam.boofcv.MAVDepthVisualOdometry;
import com.comino.slam.boofcv.vio.FactoryMAVOdometryVIO;
import com.comino.slam.boofcv.vio.tracker.FactoryMAVPointTrackerTwoPassVIO;
import com.comino.slam.boofcv.vo.FactoryMAVOdometry;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.alg.distort.DoNothingPixelTransform_F32;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.alg.tracker.klt.PkltConfig;
import boofcv.core.image.ConvertImage;
import boofcv.factory.feature.tracker.FactoryPointTrackerTwoPass;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.embed.swing.SwingFXUtils;
import javafx.scene.Scene;
import javafx.scene.image.ImageView;
import javafx.scene.image.WritableImage;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.FlowPane;
import javafx.stage.Stage;

public class StreamRealSenseTestVIO extends Application  {

	private BufferedImage output;
	private final ImageView ivrgb = new ImageView();
	private WritableImage wirgb;

	private StreamRealSenseVisDepth realsense;

	private GrayU8 gray 			= null;
	private Se3_F64 pose            = new Se3_F64();
	private double[] visAttitude    = new double[3];

	private long oldTimeDepth=0;
	private long tms = 0;

	private int mouse_x;
	private int mouse_y;

	private OptPos position = new OptPos();

	@Override
	public void start(Stage primaryStage) {
		primaryStage.setTitle("BoofCV RealSense Demo");

		FlowPane root = new FlowPane();

		root.getChildren().add(ivrgb);


		ivrgb.setOnMouseMoved(event -> {
			MouseEvent ev = event;
			mouse_x = (int)ev.getX();
			mouse_y = (int)ev.getY();
		});


		RealSenseInfo info = new RealSenseInfo(320,240, RealSenseInfo.MODE_RGB);
	//	RealSenseInfo info = new RealSenseInfo(640,480, RealSenseInfo.MODE_RGB);

		gray = new GrayU8(info.width,info.height);

		try {

		realsense = new StreamRealSenseVisDepth(0,info);

		} catch(Exception e) {
			System.out.println("REALSENSE:"+e.getMessage());
			return;
		}

		mouse_x = info.width/2;
		mouse_y = info.height/2;

		primaryStage.setScene(new Scene(root, info.width,info.height));
		primaryStage.show();


		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{1, 2, 4, 8};
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<GrayU8> tracker =
				FactoryMAVPointTrackerTwoPassVIO.klt(configKlt, new ConfigGeneralDetector(600, 3, 1),
						GrayU8.class, GrayS16.class);

		DepthSparse3D<GrayU16> sparseDepth = new DepthSparse3D.I<GrayU16>(1e-3);

		// declares the algorithm
		MAVDepthVisualOdometry<GrayU8,GrayU16> visualOdometry =
				FactoryMAVOdometryVIO.depthPnP(1.5, 80, 3, 200, 50, true,
						sparseDepth, tracker, GrayU8.class, GrayU16.class);

		visualOdometry.setCalibration(realsense.getIntrinsics(),new DoNothingPixelTransform_F32());

		Runtime.getRuntime().addShutdownHook(new Thread() {
			public void run() {
				realsense.stop();
			}
		});


		output = new BufferedImage(info.width, info.height, BufferedImage.TYPE_INT_RGB);
		wirgb = new WritableImage(info.width, info.height);
		ivrgb.setImage(wirgb);

		visualOdometry.reset();


		realsense.registerListener(new Listener() {

			int fps; float mouse_depth; float md; int mc; int mf=0; int fpm;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {


				if((System.currentTimeMillis() - tms) > 250) {
					tms = System.currentTimeMillis();
					if(mf>0)
					  fps = fpm/mf;
					if(mc>0)
					   mouse_depth = md / mc;
					mc = 0; md = 0; mf=0; fpm=0;
				}
				mf++;
				fpm += (int)(1f/((timeRgb - oldTimeDepth)/1000f)+0.5f);
				oldTimeDepth = timeRgb;

				ConvertImage.average(rgb, gray);

				if( !visualOdometry.process(gray, depth, pose) ) {
		    		 visualOdometry.reset();
					return;
				}


				pose.set(visualOdometry.getCameraToWorld());
				Vector3D_F64 T = pose.getT();

				ConvertRotation3D_F64.matrixToEuler(pose.getRotation(), EulerType.ZXY, visAttitude);


				AccessPointTracks3D points = (AccessPointTracks3D)visualOdometry;
				ConvertBufferedImage.convertTo_U8(rgb, output, false);

				Graphics c = output.getGraphics();

				int count = 0; float total = 0;  int dx=0, dy=0; int dist=999;
				int x, y; int index = -1;

				for( int i = 0; i < points.getAllTracks().size(); i++ ) {
					if(points.isInlier(i)) {


					c.setColor(Color.WHITE);

					x = (int)points.getAllTracks().get(i).x;
					y = (int)points.getAllTracks().get(i).y;



					int d = depth.get(x,y);
					if(d > 0) {


						int di = (int)Math.sqrt((x-mouse_x)*(x-mouse_x) + (y-mouse_y)*(y-mouse_y));
						if(di < dist) {
							index = i;
							dx = x;
							dy = y;
							dist = di;
						}
						total++;
						if(d<500) {
							c.setColor(Color.RED); count++;
						}
						c.drawRect(x,y, 1, 1);
					}
					}
				}

				if(depth!=null) {
				//	if(index > -1)
					//   System.out.println(visualOdometry.getTrackLocation(index));

					mc++;
					md = md + depth.get(dx,dy) / 1000f;
					c.setColor(Color.WHITE);
					c.drawOval(dx-3,dy-3, 6, 6);
				}


				c.setColor(Color.CYAN);
				c.drawString("Fps:"+fps, 10, 20);
				c.drawString(String.format("Quality:  %3.0f",visualOdometry.getQuality()), info.width-85, 20);
				c.drawString(String.format("Loc: %4.2f %4.2f %4.2f", T.x, T.y, T.z), 10, info.height-10);
				c.drawString(String.format("Depth: %3.2f", mouse_depth), info.width-85, info.height-10);
				c.drawString(String.format("ROLL:  %3.0f°", MSPMathUtils.fromRad(visAttitude[0])), info.width-85, info.height-55);
				c.drawString(String.format("PITCH: %3.0f°", MSPMathUtils.fromRad(visAttitude[1])), info.width-85, info.height-40);
				c.drawString(String.format("YAW:   %3.0f°", MSPMathUtils.fromRad(visAttitude[2])), info.width-85, info.height-25);

				position.x = T.x;
				position.y = T.y;
				position.z = T.z;

				position.tms = timeRgb;

//				if((count / total)>0.6f) {
//					c.setColor(Color.RED);
//					c.drawString("WARNING!", info.width-70, 20);
//				}
				c.dispose();

				Platform.runLater(() -> {
					SwingFXUtils.toFXImage(output, wirgb);
				});
			}

		}).start();

	}

	@Override
	public void stop() throws Exception {
		realsense.stop();
		super.stop();
	}

	public static void main(String[] args) {


		launch(args);
	}

}
