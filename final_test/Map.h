#ifndef MAP_H
#define MAP_H

#include <opencv2/core.hpp>

#include "Obstacle.h"
#include "Dubins.h"

class Map
{
	private:
		static const int MIN_OBSTACLE_CONTOUR_SIZE = 60;

		static const int LOW_H_R   = 0;
		static const int LOW_S_R   = 85;
		static const int LOW_V_R   = 175;
		static const int HIGH_H_R  = 180;
		static const int HIGH_S_R  = 140;
		static const int HIGH_V_R  = 255;
		static const int M1_H_R    = 5;
		static const int M2_H_R    = 170;

		const cv::Mat m_img_rgb;
		std::list<Obstacle> m_obstacles;

		void find_obstacles(cv::Mat image);

	public:
		Map(cv::Mat image);
};

#endif