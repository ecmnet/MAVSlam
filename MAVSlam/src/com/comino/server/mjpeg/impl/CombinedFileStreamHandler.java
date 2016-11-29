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

package com.comino.server.mjpeg.impl;

import java.awt.image.BufferedImage;
import java.io.BufferedOutputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.io.Serializable;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.imageio.ImageIO;

import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.lquac.msg_msp_command;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.main.control.listener.IMAVLinkListener;
import com.comino.msp.model.DataModel;
import com.comino.realsense.boofcv.RealSenseInfo;
import com.comino.server.mjpeg.IMJPEGOverlayListener;
import com.comino.server.mjpeg.IVisualStreamHandler;

import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;

public class CombinedFileStreamHandler implements IVisualStreamHandler {

	private boolean isRecording     = false;
	private long    count           = 0;

	private OutputStream out  = null;


	public CombinedFileStreamHandler(RealSenseInfo info, IMAVMSPController control) {
		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_CMD_COMBINEDFILESTREAM:
					switch((int)cmd.param1) {
					case 0:
						enableRecording(false);
						break;
					case 1:
						enableRecording(true);
						break;
					}
				}
			}
		});
	}

	@Override
	public void addToStream(GrayU8 grayImage, GrayU16 depthImage, DataModel model, long tms_us) {
		if(isRecording) {
              try {
            	CombinedDataSet c = new CombinedDataSet(grayImage,depthImage,model,tms_us);
            	c.write(out);
            	System.out.println(count++);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	@Override
	public void registerOverlayListener(IMJPEGOverlayListener listener) {

	}

	private void enableRecording(boolean enable) {
		if(enable) {
			try {
				out = createOutputStream();
				count = 0;
				isRecording = true;
			} catch(Exception e) {
				e.printStackTrace();
			}
		} else {
			isRecording = false;
			try {
				if(out!=null) {
					out.flush();
					out.close();
				}
			} catch(Exception e) {
				e.printStackTrace();
			}
		}
	}

	private OutputStream createOutputStream() throws IOException {
		File f = new File("DataStream");
		if(f.exists())
			f.delete();
		f.createNewFile();
		return new BufferedOutputStream(new FileOutputStream(f));
	}

}
