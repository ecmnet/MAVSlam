package com.comino.realsense.boofcv;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import com.comino.mq.bus.MWMessageBus;
import com.comino.mq.tests.OptPos;
import com.comino.realsense.boofcv.StreamRealSenseVisDepth.Listener;
import com.comino.realsense.boofcv.odometry.FactoryRealSenseOdometry;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.abst.sfm.d3.DepthVisualOdometry;
import boofcv.alg.distort.DoNothingPixelTransform_F32;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.alg.tracker.klt.PkltConfig;
import boofcv.factory.feature.tracker.FactoryPointTrackerTwoPass;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
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

public class StreamRealSenseTest extends Application  {

	private BufferedImage output;
	private final ImageView ivrgb = new ImageView();
	private WritableImage wirgb;

	private StreamRealSenseVisDepth realsense;

	private long oldTimeDepth=0;
	private long tms = 0;

	private int mouse_x;
	private int mouse_y;

	private MWMessageBus bus1 = new MWMessageBus(1,"127.0.0.1");
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
//		RealSenseInfo info = new RealSenseInfo(640,480, RealSenseInfo.MODE_RGB);

		mouse_x = info.width/2;
		mouse_y = info.height/2;

		primaryStage.setScene(new Scene(root, info.width,info.height));
		primaryStage.show();

		realsense = new StreamRealSenseVisDepth(0,info);

		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{1, 2, 4, 8};
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<GrayU8> tracker =
				FactoryPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(900, 2, 1),
						GrayU8.class, GrayS16.class);

		DepthSparse3D<GrayU16> sparseDepth = new DepthSparse3D.I<GrayU16>(1e-3);

		// declares the algorithm
		DepthVisualOdometry<GrayU8,GrayU16> visualOdometry =
				FactoryRealSenseOdometry.depthDepthPnP(1.5, 120, 2, 200, 50, true,
						sparseDepth, tracker, GrayU8.class, GrayU16.class);

		visualOdometry.setCalibration(realsense.getIntrinsics(),new DoNothingPixelTransform_F32());


		output = new BufferedImage(info.width, info.height, BufferedImage.TYPE_USHORT_555_RGB);
		wirgb = new WritableImage(info.width, info.height);
		ivrgb.setImage(wirgb);


		realsense.start(new Listener() {

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

				if( !visualOdometry.process(rgb.getBand(0),depth) ) {
					System.out.println("VO Failed!");
					visualOdometry.reset();
					return;
				}


				Se3_F64 leftToWorld = visualOdometry.getCameraToWorld();
				Vector3D_F64 T = leftToWorld.getT();


				AccessPointTracks3D points = (AccessPointTracks3D)visualOdometry;
				ConvertBufferedImage.convertTo(rgb, output, false);

				Graphics c = output.getGraphics();

				int count = 0; float total = 0;  int dx=0, dy=0; int dist=999;
				int x, y;

				for( int i = 0; i < points.getAllTracks().size(); i++ ) {
					if(points.isInlier(i)) {
					c.setColor(Color.BLUE);

					x = (int)points.getAllTracks().get(i).x;
					y = (int)points.getAllTracks().get(i).y;

					int d = depth.get(x,y);
					if(d > 0) {


						int di = (int)Math.sqrt((x-mouse_x)*(x-mouse_x) + (y-mouse_y)*(y-mouse_y));
						if(di < dist) {
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
					mc++;
					md = md + depth.get(dx,dy) / 1000f;
					c.setColor(Color.GREEN);
					c.drawOval(dx-3,dy-3, 6, 6);
				}


				c.setColor(Color.CYAN);
				c.drawString("Fps:"+fps, 10, 20);
				c.drawString(String.format("Loc: %4.2f %4.2f %4.2f", T.x, T.y, T.z), 10, info.height-10);
				c.drawString(String.format("Depth: %3.2f", mouse_depth), info.width-85, info.height-10);

				position.x = T.x;
				position.y = T.y;
				position.z = T.z;

				position.tms = timeRgb;

				bus1.writeObject(position);

				if((count / total)>0.6f) {
					c.setColor(Color.RED);
					c.drawString("WARNING!", info.width-70, 20);
				}
				c.dispose();

				Platform.runLater(() -> {
					SwingFXUtils.toFXImage(output, wirgb);
				});
			}

		});
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
