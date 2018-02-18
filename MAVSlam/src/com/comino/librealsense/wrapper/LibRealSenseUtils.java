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

package com.comino.librealsense.wrapper;

import java.nio.IntBuffer;

import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_option;
import com.sun.jna.ptr.PointerByReference;

public class LibRealSenseUtils {

	private static final int[] depth_control_options = {
	        rs_option.RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_DECREMENT,
	        rs_option.RS_OPTION_R200_DEPTH_CONTROL_ESTIMATE_MEDIAN_INCREMENT,
	        rs_option.RS_OPTION_R200_DEPTH_CONTROL_MEDIAN_THRESHOLD,
	        rs_option.RS_OPTION_R200_DEPTH_CONTROL_SCORE_MINIMUM_THRESHOLD,
	        rs_option.RS_OPTION_R200_DEPTH_CONTROL_SCORE_MAXIMUM_THRESHOLD,
	        rs_option.RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_COUNT_THRESHOLD,
	        rs_option.RS_OPTION_R200_DEPTH_CONTROL_TEXTURE_DIFFERENCE_THRESHOLD,
	        rs_option.RS_OPTION_R200_DEPTH_CONTROL_SECOND_PEAK_THRESHOLD,
	        rs_option.RS_OPTION_R200_DEPTH_CONTROL_NEIGHBOR_THRESHOLD,
	        rs_option.RS_OPTION_R200_DEPTH_CONTROL_LR_THRESHOLD
	    };

	 private static final double[][] depth_control_presets = {
		        {5, 5, 192,  1,  512, 6, 24, 27,  7,   24}, /* (DEFAULT)   Default settings on chip. Similiar to the medium setting and best for outdoors. */
		        {5, 5,   0,  0, 1023, 0,  0,  0,  0, 2047}, /* (OFF)       Disable almost all hardware-based outlier removal */
		        {5, 5, 115,  1,  512, 6, 18, 25,  3,   24}, /* (LOW)       Provide a depthmap with a lower number of outliers removed, which has minimal false negatives. */
		        {5, 5, 185,  5,  505, 6, 35, 45, 45,   14}, /* (MEDIUM)    Provide a depthmap with a medium number of outliers removed, which has balanced approach. */
		        {5, 5, 175, 24,  430, 6, 48, 47, 24,   12}, /* (OPTIMIZED) Provide a depthmap with a medium/high number of outliers removed. Derived from an optimization function. */
		        {5, 5, 235, 27,  420, 8, 80, 70, 90,   12}, /* (HIGH)      Provide a depthmap with a higher number of outliers removed, which has minimal false positives. */
		    };


	 public static final int PRESET_DEPTH_DEFAULT 	= 0;
	 public static final int PRESET_DEPTH_OFF 		= 1;
	 public static final int PRESET_DEPTH_LOW 		= 2;
	 public static final int PRESET_DEPTH_MEDIUM 	= 3;
	 public static final int PRESET_DEPTH_OPTIMIZED  = 4;
	 public static final int PRESET_DEPTH_HIGH       = 5;


	/* Provide access to several recommend sets of depth control parameters */
	public static void rs_apply_depth_control_preset(PointerByReference dev, int preset)
	{
	    IntBuffer options = IntBuffer.wrap(depth_control_options);
	    LibRealSenseWrapper.INSTANCE.rs_set_device_options(dev,options, 10, depth_control_presets[preset], null);
	}



}
