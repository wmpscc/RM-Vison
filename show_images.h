//
// Created by harvey on 20-1-17.
//

#ifndef RM_ARMOR_SHOW_IMAGES_H
#define RM_ARMOR_SHOW_IMAGES_H
#include <opencv2/core.hpp>
#include "find_armor.h"

//
void showArmorBoxes(std::string windows_name, const cv::Mat &src, const ArmorBoxes &armor_boxes);
void showArmorBox(std::string windows_name, const cv::Mat &src, const ArmorBox &armor_box);
void showLightBlobs(std::string windows_name, const cv::Mat &src, const LightBlobs &light_blobs);
void showArmorBoxesClass(std::string window_names, const cv::Mat &src, const ArmorBoxes &boxes);
void showTrackSearchingPos(std::string window_names, const cv::Mat &src, const cv::Rect2d pos);


#endif //RM_ARMOR_SHOW_IMAGES_H
