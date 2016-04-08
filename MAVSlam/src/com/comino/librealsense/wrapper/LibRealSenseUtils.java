package com.comino.librealsense.wrapper;

import java.nio.IntBuffer;

import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_device;
import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_distortion;
import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_option;
import com.sun.jna.Memory;
import com.sun.jna.Native;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;
import com.sun.jna.ptr.DoubleByReference;
import com.sun.jna.ptr.IntByReference;
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




	/* Provide access to several recommend sets of depth control parameters */
	static void rs_apply_depth_control_preset(PointerByReference dev, int preset)
	{
	    IntBuffer options = IntBuffer.wrap(depth_control_options);
	    LibRealSenseWrapper.INSTANCE.rs_set_device_options(dev,options, 10, depth_control_presets[preset], null);
	}



}
