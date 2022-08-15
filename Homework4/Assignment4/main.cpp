#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN) {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 + 3 * std::pow(t, 2) * (1 - t) * p_2 +
                     std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    auto points = control_points;

    for (int i = 1; i < control_points.size(); i++) {
        for (int j = 0; j < control_points.size() - i; j++) {
            points[j].x = points[j].x * t + (1 - t) * points[j + 1].x;
            points[j].y = points[j].y * t + (1 - t) * points[j + 1].y;
        }
    }

    return points[0];
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    //根据要求中的提示，每得到一个曲线上的点时，我们就根据它到自己周围的3×3个像素中心的距离d来为这些像素填色以达到平滑过渡的效果
    //(每个像素的颜色是255*ratio，d的范围是[0,3/√2]，ratio的范围是[0,1]，那么ratio关于d的函数就是ratio=1-√2/3d)，
    //重复计算的点就按照该点的颜色最大值算，这样就不会在线段中间出现暗点了，按道理这样应该就能达成反走样的要求：

    for (float t = 0.0; t <= 1; t += 0.001) {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        float ratio;
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {                                                        //遍历9个像素
                if (point.y + j > 700 || point.y + j < 0 || point.x + i > 700 || point.x + i < 0)  //不处理越界像素
                    continue;
                ratio =
                    1 - sqrt(2) *
                            sqrt(pow(point.y - int(point.y + j) - 0.5, 2) + pow(point.x - int(point.x + i) - 0.5, 2)) /
                            3;  //计算ratio

                window.at<cv::Vec3b>(point.y + j, point.x + i)[1] =
                    std::fmax(window.at<cv::Vec3b>(point.y + j, point.x + i)[1], 255 * ratio);  //计算像素颜色
            }
        }
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) {
        for (auto &point : control_points) {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
