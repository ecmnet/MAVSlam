package com.comino.server.mjpeg;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.imageio.ImageIO;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;

import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU8;

public class MJPEGHandler implements HttpHandler  {

	private IMJPEGOverlayListener listener = null;
	private BufferedImage image = new BufferedImage(320, 240, BufferedImage.TYPE_BYTE_GRAY);
	private static List<GrayU8>imageByteList = new ArrayList<GrayU8>(0);
	private long start = System.currentTimeMillis();

	@Override
	public void handle(HttpExchange he) throws IOException {
		Graphics gr =  image.getGraphics();
		he.getResponseHeaders().add("content-type","multipart/x-mixed-replace; boundary=--BoundaryString");
		he.sendResponseHeaders(200, 0);
		OutputStream os = he.getResponseBody();
		while(true) {
			if(imageByteList.size() > 0) {
				ConvertBufferedImage.convertTo(imageByteList.get(0), image);
				if(listener!=null)
					listener.processOverlay(gr);
				addTimeOverlay(gr);
				os.write(("--BoundaryString\r\nContent-type: image/jpeg\r\n\r\n").getBytes());
				ImageIO.write(image, "jpg", os );
				os.write("\r\n\r\n".getBytes());
				imageByteList.remove(0);
			}
			try {
				TimeUnit.MILLISECONDS.sleep(10);
			} catch (InterruptedException e) {	}
		}
	}

	public void registerOverlayListener(IMJPEGOverlayListener listener) {
		this.listener = listener;
	}

	public static void addImage(GrayU8 bands) {
		if(imageByteList.size()>10)
			imageByteList.remove(0);
		imageByteList.add(bands);
	}

	private void addTimeOverlay(Graphics ctx) {
		ctx.drawString("Time:"+(System.currentTimeMillis()-start)+"ms", 10, 20);
	}
}
