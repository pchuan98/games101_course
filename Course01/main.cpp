#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

/**
 * @brief Get the view matrix object
 *
 * @param eye_pos
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

/**
 * @brief Get the model matrix object
 * 逐个元素地构建模型变换矩阵并返回该矩阵。
 * 在此函数中，你只需要实现三维中绕 z 轴旋转的变换矩阵，而不用处理平移与缩放。
 * @param rotation_angle
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    /**
     * | cosa -sina 0 0 |
     * | sina cosa  0 0 |
     * | 0    0     1 0 |
     * | 0    0     0 1 |
     */

    model(0, 0) = (float)cos(rotation_angle / 180.0 * MY_PI);
    model(0, 1) = (float)-sin(rotation_angle / 180.0 * MY_PI);
    model(1, 0) = (float)sin(rotation_angle / 180.0 * MY_PI);
    model(1, 1) = (float)cos(rotation_angle / 180.0 * MY_PI);

    return model;
}

/**
 * @brief Get the projection matrix object
 * 使用给定的参数逐个元素地构建透视投影矩阵并返回该矩阵。
 * @param eye_fov
 * @param aspect_ratio
 * @param zNear
 * @param zFar
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // 透视投影矩阵
    // | 1/(aspect_ratio*t)     0       0                       0 |
    // | 0                      1/t     0                       0 |
    // | 0                      0       (n+f)/(n-f)     2nf/(n-f) |
    // | 0                      0       -1                      1 |

    float t = float(tan(eye_fov / 2.0 / 180.0 * MY_PI));
    float n = zNear;
    float f = zFar;

    projection(0, 0) = float(1.0 / (aspect_ratio * t));
    projection(1, 1) = float(1.0 / t);
    projection(2, 2) = (n + f) / (n - f);
    projection(2, 3) = float(2.0 * n * f / (n - f));
    projection(3, 2) = -1.0;

    return projection;
}

/**
 * @brief Get the rotation object
 * 得到绕任意过原点的轴的旋转变换矩阵。
 * @param axis
 * @param angle
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

    return rotation;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1f, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1f, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::stringstream stream;
        float angle_mod = fmod(angle, 360.0f);
        stream << std::fixed << std::setprecision(2) << angle_mod;
        cv::setWindowTitle("image", "image\t" + stream.str());

        std::cout << "key: " << key << '\n';
        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
