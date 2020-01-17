#include <iostream>
#include <opencv2/opencv.hpp>
#include "find_armor.h"
#include "constants.h"
#include "time.h"
#include <iostream>
using namespace std;
using namespace cv;

int main() {
    VideoCapture capture("./2.mp4");
    uint8_t enemy_color = ENEMY_BLUE;
    ArmorFinder armorFinder(enemy_color);
    Mat frame;
    clock_t start, finish;
    double duration;
    while (true) {
        capture >> frame;
        start = clock();
        armorFinder.run(frame);
        finish = clock();
        duration = (double)(finish - start) / CLOCKS_PER_SEC;
        cout <<"sec:" << duration << " FPS:" << 1.0 / duration << endl;
//        imshow("frame", frame);
//        waitKey(1);
    }

    return 0;
}