//
// Created by harvey on 20-1-17.
//

#include "find_armor.h"
#include <opencv2/opencv.hpp>
#include "show_images.h"

using namespace cv;

std::map<int, string> id2name = {                               //装甲板id到名称的map
        {-1, "OO"},
        {0,  "NO"},
        {1,  "B1"},
        {2,  "B2"},
        {3,  "B3"},
        {4,  "B4"},
        {5,  "B5"},
        {6,  "B7"},
        {7,  "B8"},
        {8,  "R1"},
        {9,  "R2"},
        {10, "R3"},
        {11, "R4"},
        {12, "R5"},
        {13, "R7"},
        {14, "R8"},
};

std::map<string, int> name2id = {                               //装甲板名称到id的map
        {"OO", -1},
        {"NO", 0},
        {"B1", 1},
        {"B2", 2},
        {"B3", 3},
        {"B4", 4},
        {"B5", 5},
        {"B7", 6},
        {"B8", 7},
        {"R1", 8},
        {"R2", 9},
        {"R3", 10},
        {"R4", 11},
        {"R5", 12},
        {"R7", 13},
        {"R8", 14},
};

std::map<string, int> prior_blue = {
        {"B8", 0},
        {"B1", 1},
        {"B3", 2},
        {"B4", 2},
        {"B5", 2},
        {"B7", 3},
        {"B2", 4},
        {"R8", 5},
        {"R1", 6},
        {"R3", 7},
        {"R4", 7},
        {"R5", 7},
        {"R7", 8},
        {"R2", 9},
        {"NO", 10},
};

std::map<string, int> prior_red = {
        {"R8", 0},
        {"R1", 1},
        {"R3", 2},
        {"R4", 2},
        {"R5", 2},
        {"R7", 3},
        {"R2", 4},
        {"B8", 5},
        {"B1", 6},
        {"B3", 7},
        {"B4", 7},
        {"B5", 7},
        {"B7", 8},
        {"B2", 9},
        {"NO", 10},
};

static double getPointLength(const cv::Point2f &p) {
    return sqrt(p.x * p.x + p.y * p.y);
}

ArmorFinder::ArmorFinder(uint8_t &color) :
        enemy_color(color),
        state(STANDBY_STATE),
        anti_switch_cnt(0),
        contour_area(0),
        tracking_cnt(0) {
}

bool ArmorFinder::stateSearchingTarget(cv::Mat &src) {
    if (findArmorBox(src, target_box)) { // 在原图中寻找目标，并返回是否找到
        if (last_box.rect != cv::Rect2d() &&
            (getPointLength(last_box.getCenter() - target_box.getCenter()) > last_box.rect.height * 2.0) &&
            anti_switch_cnt++ < 3) { // 判断当前目标和上次有效目标是否为同一个目标
            target_box = ArmorBox(); // 并给３帧的时间，试图找到相同目标
//            LOGM("anti-switch!");    // 即刚发生目标切换内的３帧内不发送目标位置
            return false;            // 可以一定程度避免频繁多目标切换
        } else {
            anti_switch_cnt = 0;
            return true;
        }
    } else {
        target_box = ArmorBox();
        anti_switch_cnt++;
        return false;
    }
}

bool ArmorFinder::stateTrackingTarget(cv::Mat &src) {
    auto pos = target_box.rect;
    if (!tracker->update(src, pos)) { // 使用KCFTracker进行追踪
        target_box = ArmorBox();
//        LOGW("Track fail!");
        return false;
    }
    if ((pos & cv::Rect2d(0, 0, 640, 480)) != pos) {
        target_box = ArmorBox();
//        LOGW("Track out range!");
        return false;
    }

    // 获取相较于追踪区域两倍长款的区域，用于重新搜索，获取灯条信息
    cv::Rect2d bigger_rect;
    bigger_rect.x = pos.x - pos.width / 2.0;
    bigger_rect.y = pos.y - pos.height / 2.0;
    bigger_rect.height = pos.height * 2;
    bigger_rect.width = pos.width * 2;
    bigger_rect &= cv::Rect2d(0, 0, 640, 480);
    cv::Mat roi = src(bigger_rect).clone();

    ArmorBox box;
    // 在区域内重新搜索。
    if (findArmorBox(roi, box)) { // 如果成功获取目标，则利用搜索区域重新更新追踪器
        target_box = box;
        target_box.rect.x += bigger_rect.x; //　添加roi偏移量
        target_box.rect.y += bigger_rect.y;
        for (auto &blob : target_box.light_blobs) {
            blob.rect.center.x += bigger_rect.x;
            blob.rect.center.y += bigger_rect.y;
        }
        tracker = TrackerToUse::create();
        tracker->init(src, target_box.rect);
    } else {    // 如果没有成功搜索目标，则使用判断是否跟丢。
        roi = src(pos).clone();
        if (false) { // 分类器可用，使用分类器判断。
//            cv::resize(roi, roi, cv::Size(48, 36));
//            if(classifier(roi) == 0){
//                target_box = ArmorBox();
//                LOGW("Track classify fail range!");
//                return false;
//            }
        } else { //　分类器不可用，使用常规方法判断
            cv::Mat roi_gray;
            cv::cvtColor(roi, roi_gray, CV_RGB2GRAY);
            cv::threshold(roi_gray, roi_gray, 180, 255, cv::THRESH_BINARY);
            contour_area = cv::countNonZero(roi_gray);
            if (abs(cv::countNonZero(roi_gray) - contour_area) > contour_area * 0.3) {
                target_box = ArmorBox();
                return false;
            }
        }
        target_box.rect = pos;
        target_box.light_blobs.clear();
    }
    return true;
}

bool ArmorFinder::stateStandBy() {
    state = SEARCHING_STATE;
    return true;
}

bool ArmorFinder::sendBoxPosition(uint16_t shoot_delay) {
    if (target_box.rect == cv::Rect2d()) return false;
    if (shoot_delay) {
//        LOGM(STR_CTR(WORD_BLUE, "next box %dms"), shoot_delay);
    }
    auto rect = target_box.rect;
    double dx = rect.x + rect.width / 2 - IMAGE_CENTER_X;
    double dy = rect.y + rect.height / 2 - IMAGE_CENTER_Y;
    double yaw = atan(dx / FOCUS_PIXAL) * 180 / PI;
    double pitch = atan(dy / FOCUS_PIXAL) * 180 / PI;
    double dist = DISTANCE_HEIGHT / rect.height;
//    return sendTarget(serial, yaw, -pitch, dist, shoot_delay);
}

void ArmorFinder::run(cv::Mat &src) {
//    stateSearchingTarget(src);                    // for debug
//    goto end;
    switch (state) {
        case SEARCHING_STATE:
            if (stateSearchingTarget(src)) {
                if ((target_box.rect & cv::Rect2d(0, 0, 640, 480)) == target_box.rect) { // 判断装甲板区域是否脱离图像区域
                    cv::Mat roi = src(target_box.rect).clone(), roi_gray;  /* 就使用装甲区域亮点数判断是否跟丢 */
                    cv::cvtColor(roi, roi_gray, CV_RGB2GRAY);
                    cv::threshold(roi_gray, roi_gray, 180, 255, cv::THRESH_BINARY);
                    contour_area = cv::countNonZero(roi_gray);
                    tracker = TrackerToUse::create();                       // 成功搜寻到装甲板，创建tracker对象
                    tracker->init(src, target_box.rect);
                    state = TRACKING_STATE;
                    tracking_cnt = 0;
//                    LOGM(STR_CTR(WORD_LIGHT_CYAN, "into track"));
                }
            }
            break;
        case TRACKING_STATE:
            if (!stateTrackingTarget(src) || ++tracking_cnt > 100) {    // 最多追踪100帧图像
                state = SEARCHING_STATE;
//                LOGM(STR_CTR(WORD_LIGHT_YELLOW, "into search!"));
            }
            break;
        case STANDBY_STATE:
        default:
            stateStandBy(); // currently meaningless
    }
    end:
    if (target_box.rect != cv::Rect2d()) {
        anti_top_cnt = 0;
//        time_seq.clear();
        angle_seq.clear();
        sendBoxPosition(0);
    }

    if (target_box.rect != cv::Rect2d()) {
        last_box = target_box;
    }
    bool show_armor_box = true;
    if (show_armor_box) {                 // 根据条件显示当前目标装甲板
        showArmorBox("box", src, target_box);
        cv::waitKey(1);
    }
}

