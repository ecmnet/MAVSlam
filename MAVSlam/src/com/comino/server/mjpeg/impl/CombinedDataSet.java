package com.comino.server.mjpeg.impl;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.io.Serializable;

import com.comino.msp.model.DataModel;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;

public class CombinedDataSet implements Serializable {

	private static final long serialVersionUID = -4516830726585551719L;

	private byte[]    gray;
	private short[]   depth;
	private DataModel model;
	private long      tms_us;
	private int       width;
	private int       height;


	public CombinedDataSet(GrayU8 g, GrayU16 d, DataModel m, long t) {
		this.depth      = d.data;
        this.gray       = g.data;
		this.model      = m;
		this.tms_us     = t;
		this.width      = g.width;
		this.height     = g.height;
	}

	public CombinedDataSet() {

	}

	public void write(OutputStream o) throws IOException {
		ByteArrayOutputStream ob = new ByteArrayOutputStream();
		ObjectOutputStream out = new ObjectOutputStream(ob);
		out.writeObject(this);
		ob.writeTo(o);
		out.close();
	}

	public void read(InputStream in) throws IOException, ClassNotFoundException {
		ObjectInputStream i = new ObjectInputStream(in);
		CombinedDataSet data = (CombinedDataSet) i.readObject();
		this.gray = data.gray;
		this.depth = data.depth;
		this.model = data.model;
		this.tms_us = data.tms_us;
		this.width = data.width;
		this.height = data.height;
	}

	public GrayU8 getGray() {
		GrayU8 g = new GrayU8(width,height);
		g.setData(gray);
		return g;
	}

	public GrayU16 getDepth() {
		GrayU16 g = new GrayU16(width,height);
		g.setData(depth);
		return g;
	}

	public DataModel getModel() {
		return model;
	}

	public long getTmsUs() {
		return tms_us;
	}

}
