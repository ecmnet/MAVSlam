package com.comino.realsense.boofcv;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import com.comino.realsense.boofcv.StreamRealSenseRGBDepth.Listener;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.tracker.PointTrack;
import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.abst.sfm.d3.DepthVisualOdometry;
import boofcv.alg.distort.DoNothingPixelTransform_F32;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.alg.tracker.klt.PkltConfig;
import boofcv.factory.feature.tracker.FactoryPointTrackerTwoPass;
import boofcv.factory.sfm.FactoryVisualOdometry;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.calib.VisualDepthParameters;
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
import javafx.scene.layout.FlowPane;
import javafx.stage.Stage;

public class StreamRealSenseTest extends Application  {

	private BufferedImage output;
	private final ImageView ivrgb = new ImageView();
	private WritableImage wirgb;

	private StreamRealSenseRGBDepth realsense;

	private long oldTimeDepth=0;
	private long tms = 0;

	@Override
	public void start(Stage primaryStage) {
		primaryStage.setTitle("BoofCV RealSense Demo");

		FlowPane root = new FlowPane();

		root.getChildren().add(ivrgb);

		RealSenseInfo info = new RealSenseInfo(320,240);

		primaryStage.setScene(new Scene(root, info.width,info.height));
		primaryStage.show();

		realsense = new StreamRealSenseRGBDepth(0,info);

		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{1, 2, 4, 8};
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<GrayU8> tracker =
				FactoryPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(600, 2, 0.875f),
						GrayU8.class, GrayS16.class);

		DepthSparse3D<GrayU16> sparseDepth = new DepthSparse3D.I<GrayU16>(1e-3);

		// declares the algorithm
		DepthVisualOdometry<GrayU8,GrayU16> visualOdometry =
				FactoryVisualOdometry.depthDepthPnP(1.5, 120, 2, 200, 50, true,
						sparseDepth, tracker, GrayU8.class, GrayU16.class);

		visualOdometry.setCalibration(realsense.getIntrinsics(),new DoNothingPixelTransform_F32());


		output = new BufferedImage(info.width, info.height, BufferedImage.TYPE_USHORT_555_RGB);
		wirgb = new WritableImage(info.width, info.height);
		ivrgb.setImage(wirgb);


		realsense.start(new Listener() {

			int fps;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {


				if((System.currentTimeMillis() - tms) > 1000) {
					tms = System.currentTimeMillis();
					fps = (int)(1f/((timeRgb - oldTimeDepth)/1000f)+0.5f);
				}
				oldTimeDepth = timeRgb;

				if( !visualOdometry.process(rgb.getBand(0),depth) ) {
					System.out.println("VO Failed!");
					visualOdometry.reset();
				}



				Se3_F64 leftToWorld = visualOdometry.getCameraToWorld();
				Vector3D_F64 T = leftToWorld.getT();

				AccessPointTracks3D points = (AccessPointTracks3D)visualOdometry;
				ConvertBufferedImage.convertTo(rgb, output, false);
				Graphics c = output.getGraphics();

				int count = 0; float total = 0;
				for( int i = 0; i < points.getAllTracks().size(); i++ ) {
					c.setColor(Color.BLUE);
					if(points.isInlier(i)) {
						total++;
						if(depth.get((int)points.getAllTracks().get(i).x, (int)points.getAllTracks().get(i).y)<500) {
							c.setColor(Color.RED); count++;
						}
						c.drawRect((int)points.getAllTracks().get(i).x, (int)points.getAllTracks().get(i).y, 1, 1);
					}
				}
				c.setColor(Color.CYAN);
				c.drawString("Fps:"+fps, 10, 20);
				c.drawString(String.format("Loc %8.2f %8.2f %8.2f", T.x, T.y, T.z,count), 10, info.height-10);

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
