// trax_server_KCF.cpp : Defines the entry point for the console application.
//

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "KCF/KCF_MS/kcftracker.hpp"

#include "trax.h"
#include "trax/opencv.hpp"
int main(int argc, char** argv) {

	KCFTracker kcfms_tracker;

	trax::Server handle(trax::Configuration(TRAX_REGION_RECTANGLE, TRAX_IMAGE_ANY, "KCF", "KCF", "KCF_MS"), trax_no_log);

	while (true) {

		trax::Image image;
		trax::Region region;
		trax::Properties properties;

		int tr = handle.wait(image, region, properties);
		if (tr == TRAX_INITIALIZE) {

			kcfms_tracker.init(trax::region_to_rect(region), trax::image_to_mat(image));
			//tracker.init(trax::image_to_mat(image), trax::region_to_rect(region));

			handle.reply(region, trax::Properties());

		}
		else if (tr == TRAX_FRAME) {

			cv::Rect result = kcfms_tracker.update(image_to_mat(image));
			//cv::Rect result = tracker.update(image_to_mat(image));
			handle.reply(trax::rect_to_region(result), trax::Properties());

		}
		else break;
	}

	return 1;
}



