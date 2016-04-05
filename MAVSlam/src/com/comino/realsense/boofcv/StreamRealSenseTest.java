package com.comino.realsense.boofcv;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.text.DecimalFormat;
import java.util.List;

import com.comino.librealsense.wrapper.LibRealSenseWrapper;
import com.comino.realsense.boofcv.StreamRealSenseRGBDepth.Listener;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.tracker.PointTrack;
import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.abst.sfm.d3.DepthVisualOdometry;
import boofcv.abst.sfm.d3.VisualOdometry;
import boofcv.alg.distort.DoNothingPixelTransform_F32;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.alg.tracker.klt.PkltConfig;
import boofcv.factory.feature.tracker.FactoryPointTrackerTwoPass;
import boofcv.factory.sfm.FactoryVisualOdometry;
import boofcv.gui.ListDisplayPanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.calib.VisualDepthParameters;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;
import javafx.application.Application;
import javafx.embed.swing.SwingFXUtils;
import javafx.scene.Scene;
import javafx.scene.image.ImageView;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;

public class StreamRealSenseTest extends Application  {

	private BufferedImage output;
	private BufferedImage output_d;
	private final ImageView ivrgb = new ImageView();
	private WritableImage wirgb;

	private final ImageView ivdepth = new ImageView();
	private Planar<GrayU8> gray	 = new Planar<GrayU8>(GrayU8.class,1,1,3);
	private WritableImage widepth;

	private StreamRealSenseRGBDepth realsense;

	private long oldTimeDepth=0;
	private long tms = 0;

	@Override
	public void start(Stage primaryStage) {
		primaryStage.setTitle("BoofCV RealSense Demo");

		FlowPane root = new FlowPane();

		root.getChildren().add(ivrgb);
		root.getChildren().add(ivdepth);

		RealSenseInfo info = new RealSenseInfo(320,240);

		primaryStage.setScene(new Scene(root, info.width,info.height));
		primaryStage.show();

//		gray.reshape(info.width, info.height);

		realsense = new StreamRealSenseRGBDepth();

		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{1, 2, 4, 8};
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<GrayU8> tracker =
				FactoryPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(600, 2, 1f),
						GrayU8.class, GrayS16.class);

		DepthSparse3D<GrayU16> sparseDepth = new DepthSparse3D.I<GrayU16>(1e-3);

		// declares the algorithm
		DepthVisualOdometry<GrayU8,GrayU16> visualOdometry =
				FactoryVisualOdometry.depthDepthPnP(1.5, 120, 2, 200, 50, true,
						sparseDepth, tracker, GrayU8.class, GrayU16.class);

		VisualDepthParameters param = UtilIO.loadXML(getClass().getResource("visualdepth.xml"));
		visualOdometry.setCalibration(param.getVisualParam(),new DoNothingPixelTransform_F32());


		output = new BufferedImage(info.width, info.height, BufferedImage.TYPE_USHORT_555_RGB);
//		output_d = new BufferedImage(info.width, info.height, BufferedImage.TYPE_USHORT_555_RGB);
		wirgb = new WritableImage(info.width, info.height);
		ivrgb.setImage(wirgb);

//		widepth = new WritableImage(info.width, info.height);
//		ivdepth.setImage(widepth);



		realsense.start(0, info, new Listener() {

			int fps; Vector3D_F64 old; Point3D_F64 p;

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
				if(old==null)
					old = T.copy();



//				if(old!=null)
//				System.out.printf("Speed    %8.2f %8.2f %8.2f   \n ", (T.x-old.x)*30,(T.y-old.y)*30,(T.z-old.z)*30);
//				System.out.printf("Loc %8.2f %8.2f %8.2f   \n", T.x, T.y, T.z);

//				System.out.printf("Location %8.2f\n", T.y);


				List<PointTrack> points = tracker.getAllTracks(null);
				ConvertBufferedImage.convertTo(rgb, output, false);
				Graphics c = output.getGraphics();
				int N = points.size();
				for( int i = 0; i < N; i++ ) {
					c.drawRect((int)points.get(i).x, (int)points.get(i).y, 1, 1);
				}
				c.setColor(Color.CYAN);
				c.drawString("Fps:"+fps, 10, 20);
				c.drawString(String.format("Loc %8.2f %8.2f %8.2f", T.x, T.y, T.z), 10, info.height-10);
				c.dispose();

				old = T.copy();

				SwingFXUtils.toFXImage(output, wirgb);

//				makeDepthHistogram(depth,gray);
//				SwingFXUtils.toFXImage(output_d, widepth);



			}

		});
	}


	private void makeDepthHistogram(GrayU16 in, Planar<GrayU8> output) {

		GrayU8 band0 = output.getBand(0);
		GrayU8 band1 = output.getBand(1);
		GrayU8 band2 = output.getBand(2);

		for(int i = 0;i<in.height*in.width;i++) {
			band0.getData()[i] = (byte)(in.data[i]);
			band1.getData()[i] = (byte)(in.data[i] / 128);
		}

	}

	private String inlierPercent(VisualOdometry alg) {
		if( !(alg instanceof AccessPointTracks3D))
			return "";

		AccessPointTracks3D access = (AccessPointTracks3D)alg;

		int count = 0;
		int N = access.getAllTracks().size();
		for( int i = 0; i < N; i++ ) {
			if( access.isInlier(i) )
				count++;
		}

		return String.format("%%%5.3f", 100.0 * count / N);
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
