package com.comino.realsense.boofcv;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;

import com.comino.librealsense.wrapper.LibRealSenseWrapper;
import com.comino.realsense.boofcv.StreamRealSenseRgbDepth.Listener;

import boofcv.gui.ListDisplayPanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import javafx.application.Application;
import javafx.embed.swing.SwingFXUtils;
import javafx.scene.Scene;
import javafx.scene.image.ImageView;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.StackPane;
import javafx.stage.Stage;

public class StreamRealSenseTest extends Application  {

	private BufferedImage output;
	private final ImageView imv = new ImageView();
	private WritableImage wi;

	private StreamRealSenseRgbDepth realsense;

	private long oldTimeDepth=0;
	private long tms = 0;

	@Override
	public void start(Stage primaryStage) {
		primaryStage.setTitle("Hello World!");

		StackPane root = new StackPane();

		root.getChildren().add(imv);

		primaryStage.setScene(new Scene(root, 800,600));
		primaryStage.show();

		RealSenseInfo info = new RealSenseInfo(480,360);

		output = new BufferedImage(info.width, info.height, BufferedImage.TYPE_USHORT_565_RGB);
		wi = new WritableImage(info.width, info.height);
		imv.setImage(wi);

		realsense = new StreamRealSenseRgbDepth();
		realsense.start(0, info, new Listener() {

			String fps;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {
				ConvertBufferedImage.convertTo(depth, output, false);

				if((System.currentTimeMillis() - tms) > 1000) {
					tms = System.currentTimeMillis();
					fps = "FPS: "+(int)(1f/((timeDepth - oldTimeDepth)/1000f)+0.5f);
				}
				oldTimeDepth = timeDepth;

				Graphics c = output.getGraphics();
				c.setColor(Color.CYAN);
				c.drawString(fps, 10, 20);
				c.dispose();

				SwingFXUtils.toFXImage(output, wi);
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
